#ifndef AR9311_V4L2_H
#define AR9311_V4L2_H

/** Open the V4L2 node whose sysfs name is @p name (O_RDWR | O_NONBLOCK).
 *  Nodes are matched by name because /dev/videoN depends on module probe
 *  order.  Returns the fd, or -1 with the reason on stderr. */
int ar9311_v4l2_open_by_name(const char *name);

#endif /* AR9311_V4L2_H */
