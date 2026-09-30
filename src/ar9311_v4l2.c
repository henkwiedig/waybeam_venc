#include "ar9311_v4l2.h"

#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int ar9311_v4l2_open_by_name(const char *name)
{
	DIR *dir = opendir("/sys/class/video4linux");
	struct dirent *de;
	int found = 0;
	int fd = -1;

	if (!dir) {
		perror("[ar9311] /sys/class/video4linux");
		return -1;
	}
	while ((de = readdir(dir)) != NULL) {
		char path[300];
		char node[64] = "";
		FILE *f;

		if (strncmp(de->d_name, "video", 5) != 0)
			continue;
		snprintf(path, sizeof(path), "/sys/class/video4linux/%s/name",
			de->d_name);
		f = fopen(path, "r");
		if (!f)
			continue;
		if (fgets(node, sizeof(node), f))
			node[strcspn(node, "\r\n")] = '\0';
		fclose(f);
		if (strcmp(node, name) != 0)
			continue;
		found = 1;
		snprintf(path, sizeof(path), "/dev/%s", de->d_name);
		fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0)
			perror("[ar9311] open video node");
		break;
	}
	closedir(dir);
	if (!found)
		fprintf(stderr, "ERROR: [ar9311] no \"%s\" video node (modules not "
			"loaded? see load-ar9311-camera)\n", name);
	return fd;
}
