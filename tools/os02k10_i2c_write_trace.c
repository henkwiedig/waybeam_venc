/*
 * os02k10_i2c_write_trace.c — LD_PRELOAD shim that logs every OS02K10
 * register write ar_ldyhs_sky performs, without patching the binary.
 *
 * os02k10_write_register() is a local (non-PLT) function statically linked
 * into ar_ldyhs_sky, so it can't be interposed directly. But the actual I2C
 * bytes still cross the libc boundary: open()/ioctl()/write() are all
 * dynamically bound (NEEDED libc.so). This watches for
 *   open("/dev/i2c-0", ...)                      -> remembers the fd
 *   ioctl(fd, I2C_SLAVE_FORCE or I2C_SLAVE, 0x36) -> confirms it's the sensor
 *   write(fd, {reg_hi, reg_lo, val}, 3)           -> logs it
 * and passes every call through to the real libc function unchanged.
 *
 * Style follows tools/mi_snr_preload_trace.c (same trace_log/RTLD_NEXT
 * pattern) applied to a syscall-level interposition instead of an MI_SNR_*
 * one, because the OS02K10 driver in this firmware has no dynamic symbols
 * of its own to hook (see documentation/os02k10/OS02K10_ASCENT_REGISTER_RE.md
 * section 9).
 *
 * DEV-ONLY. Push to /tmp, LD_PRELOAD it around a real boot, delete it
 * afterwards. Nothing is patched or installed; this is a pure env-var
 * interposition and has no effect once unset.
 *
 * Build:
 *   arm-openipc-linux-musleabi-gcc -O2 -fPIC -shared -o out/os02k10_i2c_write_trace.so \
 *       tools/os02k10_i2c_write_trace.c -ldl
 *
 * Run (capture a real init sequence during boot):
 *   scp -O out/os02k10_i2c_write_trace.so root@<ascent-ip>:/tmp/
 *   ssh root@<ascent-ip> \
 *     'LD_PRELOAD=/tmp/os02k10_i2c_write_trace.so /fpv/ar_ldyhs_sky' \
 *     2>/tmp/os02k10_boot_writes.log
 *
 * Output lines: "<mono_seconds> os02k10 WRITE 0xREG 0xVAL fd=<n>"
 */

#define _GNU_SOURCE

#include <dlfcn.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define I2C_SLAVE       0x0703
#define I2C_SLAVE_FORCE 0x0706
#define OS02K10_ADDR    0x36

#define MAX_TRACKED_FD 256

static int g_is_i2c_fd[MAX_TRACKED_FD];    /* opened "/dev/i2c-N" */
static int g_is_target_fd[MAX_TRACKED_FD]; /* + bound to 0x36 via ioctl */

static void trace_log(const char *fmt, ...)
{
	char line[256];
	struct timespec ts;
	va_list ap;
	int off, n;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	off = snprintf(line, sizeof(line), "%ld.%03ld ", (long)ts.tv_sec,
	               (long)(ts.tv_nsec / 1000000L));
	if (off < 0 || off >= (int)sizeof(line))
		return;

	va_start(ap, fmt);
	n = vsnprintf(line + off, sizeof(line) - (size_t)off, fmt, ap);
	va_end(ap);
	if (n < 0)
		return;

	size_t len = strnlen(line, sizeof(line));
	if (len + 1 < sizeof(line))
		line[len++] = '\n';
	(void)write(STDERR_FILENO, line, len);
}

static void *load_next(const char *symbol)
{
	void *fn = dlsym(RTLD_NEXT, symbol);
	if (!fn)
		trace_log("os02k10-trace %s resolve failed: %s", symbol, dlerror());
	return fn;
}

int open(const char *path, int flags, ...)
{
	static int (*real_open)(const char *, int, ...);
	mode_t mode = 0;
	int fd;

	if (!real_open)
		real_open = (int (*)(const char *, int, ...))load_next("open");
	if (!real_open)
		return -1;

	if (flags & O_CREAT) {
		va_list ap;
		va_start(ap, flags);
		mode = (mode_t)va_arg(ap, int);
		va_end(ap);
		fd = real_open(path, flags, mode);
	} else {
		fd = real_open(path, flags);
	}

	if (fd >= 0 && fd < MAX_TRACKED_FD && path != NULL &&
	    strncmp(path, "/dev/i2c-", 9) == 0) {
		g_is_i2c_fd[fd] = 1;
		g_is_target_fd[fd] = 0;
		trace_log("os02k10 OPEN %s fd=%d", path, fd);
	}
	return fd;
}

int ioctl(int fd, int request, ...)
{
	static int (*real_ioctl)(int, int, ...);
	va_list ap;
	void *arg;
	int ret;

	if (!real_ioctl)
		real_ioctl = (int (*)(int, int, ...))load_next("ioctl");
	if (!real_ioctl)
		return -1;

	va_start(ap, request);
	arg = va_arg(ap, void *);
	va_end(ap);

	ret = real_ioctl(fd, request, arg);

	if (ret >= 0 && fd >= 0 && fd < MAX_TRACKED_FD && g_is_i2c_fd[fd] &&
	    (request == I2C_SLAVE_FORCE || request == I2C_SLAVE)) {
		unsigned long slave_addr = (unsigned long)(uintptr_t)arg;
		g_is_target_fd[fd] = (slave_addr == OS02K10_ADDR);
		trace_log("os02k10 IOCTL fd=%d slave=0x%02lx target=%d",
		          fd, slave_addr, g_is_target_fd[fd]);
	}
	return ret;
}

ssize_t write(int fd, const void *buf, size_t count)
{
	static ssize_t (*real_write)(int, const void *, size_t);

	if (!real_write)
		real_write = (ssize_t (*)(int, const void *, size_t))load_next("write");
	if (!real_write)
		return -1;

	if (fd >= 0 && fd < MAX_TRACKED_FD && g_is_target_fd[fd] &&
	    count == 3 && buf != NULL) {
		const unsigned char *b = (const unsigned char *)buf;
		unsigned int reg = ((unsigned int)b[0] << 8) | b[1];
		trace_log("os02k10 WRITE 0x%04x 0x%02x fd=%d", reg, b[2], fd);
	}

	return real_write(fd, buf, count);
}

int close(int fd)
{
	static int (*real_close)(int);

	if (!real_close)
		real_close = (int (*)(int))load_next("close");
	if (!real_close)
		return -1;

	if (fd >= 0 && fd < MAX_TRACKED_FD) {
		g_is_i2c_fd[fd] = 0;
		g_is_target_fd[fd] = 0;
	}
	return real_close(fd);
}

__attribute__((constructor))
static void os02k10_trace_ctor(void)
{
	trace_log("os02k10 i2c write trace active");
}
