REM mingw環境前提
socat -d -d tcp-l:54321,reuseaddr,fork /dev/ttyS7,raw,b115200,nonblock