#!/bin/sh
# SPDX-License-Identifier: MIT

#set -e
#set -x
OP=$1
PLAT=$2

CONFIGFS="/sys/kernel/config"
GADGET="$CONFIGFS/usb_gadget"
VID="0x1d6b"
PID="0x0104"
SERIAL="0123456789"
MANUF=$(hostname)
PRODUCT="pizero"
#echo "pizero USB Device UVC" > $GADGET_CONFIGFS_ROOT/strings/0x409/product

BOARD=$(strings /proc/device-tree/model)

case $BOARD in
	*)
		UDC=`ls /sys/class/udc` # will identify the 'first' UDC
		UDC_ROLE=/dev/null # Not generic
		;;
esac

echo "Detecting platform:"
echo "  board : $BOARD"
echo "  udc   : $UDC"

create_eth() {

	CONFIG=$1
	FUNCTION=$2
	echo "	Creating ECM.0 (ethernet) gadget functionality : $FUNCTION"
        cd $GADGET/pizero
        mkdir -p functions/$FUNCTION
        # first byte of address must be even
        HOST="48:6f:73:74:50:43" # "HostPC"
        SELF="42:61:64:55:53:42" # "BadUSB"
        echo $HOST > functions/$FUNCTION/host_addr
        echo $SELF > functions/$FUNCTION/dev_addr
	ln -s functions/$FUNCTION configs/c.1

}
create_serial() {

	CONFIG=$1
	FUNCTION=$2
	echo "	Creating ACM (serial) gadget functionality : $FUNCTION"
        cd $GADGET/pizero
        mkdir -p functions/$FUNCTION
        # first byte of address must be even
	ln -s functions/$FUNCTION configs/c.1

}
create_frame() {
	# Example usage:
	# create_frame <function name> <width> <height> <format> <name>

	FUNCTION=$1
	WIDTH=$2
	HEIGHT=$3
	FORMAT=$4
	NAME=$5

	wdir=functions/$FUNCTION/streaming/$FORMAT/$NAME/${HEIGHT}p
	mkdir -p $wdir
        #temp new values
        #WIDTH=640
        #HEIGHT=360
	echo $WIDTH > $wdir/wWidth
	echo $HEIGHT > $wdir/wHeight
	#echo $(( $WIDTH * $HEIGHT * 1 )) > $wdir/dwMaxVideoFrameBufferSize
        # each frame is 100ns
        echo 500000 > $wdir/dwDefaultFrameInterval
	cat <<EOF > $wdir/dwFrameInterval
166666
200000
333333
500000
EOF
	echo $(( $WIDTH * $HEIGHT * 80 )) > $wdir/dwMinBitRate
	echo $(( $WIDTH * $HEIGHT * 160 )) > $wdir/dwMaxBitRate

}

create_uvc() {
	#	create_uvc <target config> <function name>
	#	create_uvc config/c.1 uvc.0
	CONFIG=$1
	FUNCTION=$2

	echo "	Creating UVC gadget functionality : $FUNCTION"
	mkdir functions/$FUNCTION

	#create_frame $FUNCTION 640 360 uncompressed u
	#create_frame $FUNCTION 736 480 uncompressed u
        case "$PLAT" in 
          test)
            create_frame $FUNCTION 1920 1080 mjpeg m
            ;;
          mjpeg)
	    create_frame $FUNCTION 1280 720 mjpeg m
          ;;
          yuyv)
	    create_frame $FUNCTION 1280 720 uncompressed u
          ;;
          all)
	    create_frame $FUNCTION 1280 720 uncompressed u
	    create_frame $FUNCTION 1280 720 mjpeg m
          ;;
          *)
            echo "unknown format, exiting"
            exit 1
	    #create_frame $FUNCTION 1280 720 uncompressed u
	    #create_frame $FUNCTION 1280 720 mjpeg m
          ;;
        esac

	mkdir functions/$FUNCTION/streaming/header/h
	cd functions/$FUNCTION/streaming/header/h
	
        ln -s ../../uncompressed/u
	ln -s ../../mjpeg/m
	cd ../../class/fs
	ln -s ../../header/h
	cd ../../class/hs
	ln -s ../../header/h
	
        cd ../../../control
	mkdir header/h
	ln -s header/h class/fs
	#ln -s header/h class/ss
	cd ../../../

	# Set the packet size: uvc gadget max size is 3k...
	#echo 3072 > functions/$FUNCTION/streaming_maxpacket .. yeah but ain't working
	echo 2048 > functions/$FUNCTION/streaming_maxpacket
#	echo 1024 > functions/$FUNCTION/streaming_maxpacket .. only if you want slow fps

	#echo 15 > functions/$FUNCTION/streaming_maxburst
        echo 1 >  functions/$FUNCTION/streaming_interval
	ln -s functions/$FUNCTION configs/c.1
}

delete_uvc() {
	#	delete_uvc <target config> <function name>
	#	delete_uvc config/c.1 uvc.0
	CONFIG=$1
	FUNCTION=$2

	echo "	Deleting UVC gadget functionality : $FUNCTION"
	#rm $CONFIG/$FUNCTION
	rmdir functions/$FUNCTION

	rm functions/$FUNCTION/control/class/*/h
	rm functions/$FUNCTION/streaming/class/*/h
	rm functions/$FUNCTION/streaming/header/h/m
	rm functions/$FUNCTION/streaming/header/h/u
	
        rmdir functions/$FUNCTION/streaming/uncompressed/u/*/
	rmdir functions/$FUNCTION/streaming/uncompressed/u
	
        rmdir functions/$FUNCTION/streaming/mjpeg/m/*/
	rmdir functions/$FUNCTION/streaming/mjpeg/m
	
        rmdir functions/$FUNCTION/streaming/header/h
	rmdir functions/$FUNCTION/control/header/h
	
        rmdir functions/$FUNCTION
        rm $CONFIG/$FUNCTION
	#rm configs/c.1/uvc.0
}
delete_ethernet(){
	CONFIG=$1
	FUNCTION=$2
        echo "function/$FUNCTION"
        rmdir functions/$FUNCTION
        rm $CONFIG/$FUNCTION
}
case "$OP" in
    start)
	echo "Creating the USB gadget"
	#echo "Loading composite module"
	#modprobe libcomposite

	echo "Creating gadget directory pizero"
	mkdir -p $GADGET/pizero

	cd $GADGET/pizero
	if [ $? -ne 0 ]; then
	    echo "Error creating usb gadget in configfs"
	    exit 1;
	else
	    echo "OK"
	fi

	echo "Setting Vendor and Product ID's"
	echo $VID > idVendor
	echo $PID > idProduct
        echo 0x0100 > bcdDevice
        echo 0x0200 > bcdUSB
        echo 0xEF > bDeviceClass
        echo 0x02 > bDeviceSubClass
        echo 0x01 > bDeviceProtocol
        echo 64 > bMaxPacketSize0
	echo "OK"

	echo "Setting English strings"
	mkdir -p strings/0x409
	echo $SERIAL > strings/0x409/serialnumber
	echo $MANUF > strings/0x409/manufacturer
	echo $PRODUCT > strings/0x409/product
	echo "OK"

	echo "Creating Config"
	mkdir configs/c.1
        echo 500 > configs/c.1/MaxPower
	mkdir configs/c.1/strings/0x409

	echo "Creating functions..."
	
        create_uvc configs/c.1 uvc.0
	create_eth configs/c.1 ecm.usb0
        #create_serial configs/c.1 acm.usb0
	echo "OK"

	echo "Binding USB Device Controller"
        udevadm settle -t 5 || :
        sleep 2
	echo $UDC > UDC
        echo peripheral > $UDC_ROLE
	cat $UDC_ROLE
	echo "OK"
	;;

    stop)
	echo "Stopping the USB gadget"
        echo "Sorry this doesn't work well"
        echo "Please reboot Pi to fully  stop USB gadget"

	set +e # Ignore all errors here on a best effort

	cd $GADGET/pizero

	if [ $? -ne 0 ]; then
	    echo "Error: no configfs gadget found"
	    exit 1;
	fi

	echo "Unbinding USB Device Controller"
	grep $UDC UDC && echo "" > UDC
	echo "OK"

	#delete_uvc configs/c.1 uvc.0
        #delete_ethernet configs/c.1 ecm.usb0

	echo "Clearing English strings"
	rmdir strings/0x409
	echo "OK"
	
        delete_uvc configs/c.1 uvc.0
        delete_ethernet configs/c.1 ecm.usb0


	echo "Cleaning up configuration"
	rmdir configs/c.1/strings/0x409
	#rm configs/c.1/uvc.0
	rmdir configs/c.1

        rmdir functions/uvc.0 #try to put this in delete_uvc
        rmdir functions/ecm.usb0
        #rm string/0x409
	echo "OK"

	echo "Removing gadget directory"
	cd $GADGET
	rmdir pizero
	cd /
	echo "OK"

	#echo "Disable composite USB gadgets"
	#modprobe -r libcomposite
	#echo "OK"
	;;
    *)
	echo "Usage : $0 {start mjpeg|start yuyv|stop}"
esac
