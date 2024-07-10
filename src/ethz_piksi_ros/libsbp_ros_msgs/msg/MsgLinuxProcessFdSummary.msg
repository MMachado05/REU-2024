# This message is automatically generated using generator.py
# PLEASE DO NOT MODIFY MANUALLY.

# Linux
# MSG_LINUX_PROCESS_FD_SUMMARY 0x7F07
#
# Summary of open file descriptors on the system
#
# Summary of open file descriptors on the system.

# count of total FDs open on the system
uint32 sys_fd_count # []

# A null delimited list of strings which alternates between
# a string representation of the process count and the file
# name whose count it being reported.  That is, in C string
# syntax "32\0/var/log/syslog\012\0/tmp/foo\0" with the end
string most_opened # []
