~/go/bin/msp-tool -p /dev/cu.usbserial-0001 | grep  --line-buffered -v "out of band" | grep  --line-buffered -v "econnect" | grep -v "Unhandled MSP frame" --line-buffered | sed -e 's/^/\r/'
