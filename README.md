﻿My goal was to operate FlightGear, controlling the simulation’s throttle position with the speed of my bike’s rear wheel, using a Raspberry Pi 5. A second goal, is two make thing simple enough that someone with only basic computer skills could repeat the feat. I am not there yet, but I have start somewhere.
I purchased a Canakit Raspberry Pi 5 from Amazon: “https://www.amazon.com/CanaKit-Raspberry-Starter-Kit-PRO/dp/B0CRSNCJ6Y/ref=sr_1_1?crid=E50QUTI3IJ0V&dib=eyJ2IjoiMSJ9.Yp-_3HVcTHw7QZ8ENnr7e8AXueED-O9udMb5LwHI2UaNJufKjE57DFwNwZ87KMdeGaBG3v5EOjT-ne-K1ffvbfA4RblLhvn-Y5hxnAOw1gM077EvbDAIET5e5hzG4EvIbIqLlrF-FVRe8T6IiQvXJ0iLU59mgiXY6CxtVXBonP5nhJ09uOemHvgRA7FrtiCdMDuoScFsE45ezmgk6uqNJfykV7Y-9fceJM_4ia3bF54.Q3VKhP-QYbpxw1V23jOq-bDk4V2iDjoZCDByqvAoQ2s&dib_tag=se&keywords=raspberry%2Bpi%2B5&qid=1724173963&sprefix=%2Caps%2C96&sr=8-1&th=1”
My rear wheel was already equipped with a Wahoo speed sensor: “https://www.amazon.com/Wahoo-Cycling-Speed-Sensor-Bluetooth/dp/B01DIE7LUG/ref=sr_1_3?crid=22BSYE0DDCLMN&dib=eyJ2IjoiMSJ9.yLk_I74wv6rfZZ3boBy2XQ6ZWsNAOkkohdlcvT6HCgOzqchESwaJOwQqmF3I0DNTOzGPPWmyeD43F7sZ7XF5kWC1u9bJp7B6Lo_quXJnAf5Mq7A94jYoNCTdm9hJnGsi6gQ01X5FjGrhHnnQ5qr1qTGxmScobpMbvhzQ1YPwMdA8b_jIy1kWqE-UfWMH_ZIuyRysuF4gHueziCuUTezWh6T1iw7SQZ4tnEZ02pyEGkMeh8P8yWaSGirP1EER12t5GSYQfATJuygWcTiUKZp6rK3ZaDP6bs1WerGHMOc8_rs.q2WxuRx1gYKLV6Jrpx2JFRUN-jFW4r-2ng9MceMlsCs&dib_tag=se&keywords=wahoo+speed+sensor&qid=1724174069&sprefix=WaHOO+SP%2Caps%2C79&sr=8-3”
The bike is fixed to a trainer stand. I built a standing desk from 8020 extrusions. I adorned the extrusions with part of an old shelf. I use a plastic cover over the keyboard. You can try my scheme without the plastic shield if you are good at not breaking a sweat. Also, I ride in the basement, facing a screen that is the Pi 5’s display next to two fans. One is for my comfort; the second is to blow sweat away from my bike’s bottom bracket. Again, if you don’t sweat you can ignore these features.
I wrote a ascript that executes a modified version of gatttool (the modified gatttool now requires elevated privileges: i.e sudo) that translate the speed of your bike’s rear wheel into a joystick throttle position.
Gatttool was created by downloading the distribution of blueZ-5.66 – “bluez-5.66.tar.xz” -→”http://www.kernel.org/pub/linux/bluetooth/bluez-5.66.tar.xz” This was expanded. From the directory formed by the expansion of bluez-5.66, I executed the following commands: 
	./configure –enable-depricated 
	sudo make 
	sudo make install
I the replaced the version of gatttool in the attrib directory. I redid: 
	./configure –enable-depricated 
	sudo make 
	sudo make install
I then copied the created version of gatttool to /usr/bin/gatttool using elevated privileges.
In its current for the new version of gatttool when executed with elevated 
privileges will form a joystick node that will adjust flightgear’s throttle.  The elevated privileges timed out after 400 seconds.  I wanted to have longer flights. If “sudo -v” is typed into a separate console window, the 400 sudo time out is no longer active.  
On September 28, 2024, I completed a virtual flight from 4J2 (Berrien County) to KMAI (Marianna Municipal) using gatttool and my bike to control the throttle.  Incluede are:
	A single screen shot taken shortly after take-off
	the fgtape from the last hour of flight
	gatttool.c, my latest version of gatttool
	the text output of gatttool from my flight
	the script I used, sixthcollectData
	a pidture showing my flight from skyvector
	the “speed_parameters.txt”, I used

I wish to openly acknowledge the help I got from the Holy Spirit. 


Please help me add omitted details from my explanation.

