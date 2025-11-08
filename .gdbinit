set print thread-events on
set breakpoint pending on
set pagination off
set follow-fork-mode child

set substitute-path /build/linux/debian_bullseye_amd64-sysroot/usr/include /usr/include
set substitute-path /build/linux/debian_bullseye_amd64-sysroot/usr/include/c++/10 /usr/include/c++/10

skip -gfi std::
skip -gfi absl::

define hookpost-attach
    set scheduler-locking step
end
