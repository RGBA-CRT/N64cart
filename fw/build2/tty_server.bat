REM mingw環境前提
@REM socat -d -d tcp-l:54321,reuseaddr,fork /dev/ttyS7,raw,b115200,nonblock
socat -d -d tcp-l:54321,reuseaddr,fork /dev/ttyS9,raw,b921600,nonblock