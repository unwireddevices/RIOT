заливать прошивку:

arm-none-eabi-gdb -x gdbinit.txt bin/unwd-range-l1-r3/lora-star-uni.hex

и затем жать reset на umdk-rf!

gdbinit.txt:

target extended-remote \\.\COM10
mon conn enable
mon swdp_scan
attach 1
load
quit

порт - тот, на котором гдб сервер

что она выдаёт:

set
00: set
set eui64 <16 hex digits> -- sets device EUI64 (permanently after save!)
	Example: set eui64 00000000000011ff
	> set eui64 00000000000011ff
	15: set eui64 00000000000011ff
	[ok] EUI64 = 0x00000000000011ff
	[config] Current configuration:
	EUI64 = 0x00000000000011ff
	> save
	24: save
	Current configuration:
	[config] Current configuration:
	EUI64 = 0x00000000000011ff
	[!] Saving current configuration...
	[ok] Configuration is written. Rebooting...
	яmain(): This is RIOT! (Version: v1.9-1713-gc050-Alex-lora-star-simple)
	*****************************************
	Unwired Range firmware by Unwired Devices
	www.unwds.com - info@unwds.com
	*****************************************
	Version: 1.10 (Mar  6 2017 17:51:59)

	Magic is wrong
	[config] No valid configuration found in NVRAM. It's either first launch or NVRAM content is corrupted.
	[config] Could you please provide APPID64 and JOINKEY for this device?
	[unk] Please configure this device. Type "help" for list of possible commands.
	[config] Current configuration:
	APPID64 = <not set>
	JOINKEY = <not set>
	DEVNONCE = <not set>
	> set appid64 0000000000000000
	41: set appid64 0000000000000000
	[ok] APPID64 = 0x0000000000000000
	[config] Current configuration:
	APPID64 = <not set>
	JOINKEY = <not set>
	DEVNONCE = <not set>
	> set joinkey 00000000000000
	53: set joinkey 00000000000000
	[error] There must be 32 hexadecimal digits in lower case as JOIN KEY
	> set joinkey 0000000000000000
	57: set joinkey 0000000000000000
	[error] There must be 32 hexadecimal digits in lower case as JOIN KEY
	> set joinkey 000000000000000
	59: set joinkey 000000000000000
	[error] There must be 32 hexadecimal digits in lower case as JOIN KEY
	> set joinkey 00000000000000000
	01:04: set joinkey 00000000000000000
	[error] There must be 32 hexadecimal digits in lower case as JOIN KEY
	> set joinkey 00000000000000000000000000000000
	01:23: set joinkey 00000000000000000000000000000000
	[ok] JOINKEY = 00000000000000000000000000000000
	[config] Current configuration:
	APPID64 = <not set>
	JOINKEY = 00000000000000000000000000000000
	DEVNONCE = <not set>
	> set devnonce abcdabcd
	01:30: set devnonce abcdabcd
	[ok] That's a nice nonce, thank you!
	 Don't forget to save it and write it down for future use, as there's no way to get it back from the programmed LoRa modem.
	  DEVNONCE = abcdabcd
	  [config] Current configuration:
	  APPID64 = <not set>
	  JOINKEY = 00000000000000000000000000000000
	  DEVNONCE = 0xABCDABCD
	  > set appid64 0000000000000000
	  01:33: set appid64 0000000000000000
	  [ok] APPID64 = 0x0000000000000000
	  [config] Current configuration:
	  APPID64 = <not set>
	  JOINKEY = 00000000000000000000000000000000
	  DEVNONCE = 0xABCDABCD
	  > set appid64 0000000000000001
	  01:36: set appid64 0000000000000001
	  [ok] APPID64 = 0x0000000000000001
	  [config] Current configuration:
	  APPID64 = 0x0000000000000001
	  JOINKEY = 00000000000000000000000000000000
	  DEVNONCE = 0xABCDABCD
	  > set joinkey 00000000000000000000000000000001
	  01:45: set joinkey 00000000000000000000000000000001
	  [ok] JOINKEY = 00000000000000000000000000000001
	  [config] Current configuration:
	  APPID64 = 0x0000000000000001
	  JOINKEY = 00000000000000000000000000000001
	  DEVNONCE = 0xABCDABCD
	  > set joinkey 11111111111111111111111111111111
	  01:57: set joinkey 11111111111111111111111111111111
	  [ok] JOINKEY = 11111111111111111111111111111111
	  [config] Current configuration:
	  APPID64 = 0x0000000000000001
	  JOINKEY = 11111111111111111111111111111111
	  DEVNONCE = 0xABCDABCD
	  > save
	  02:06: save
	  Current configuration:
	  [config] Current configuration:
	  APPID64 = 0x0000000000000001
	  JOINKEY = 11111111111111111111111111111111
	  DEVNONCE = 0xABCDABCD
	  [!] Saving current configuration...
	  [ok] Configuration is written. Rebooting...
	  яmain(): This is RIOT! (Version: v1.9-1713-gc050-Alex-lora-star-simple)
	  *****************************************
	  Unwired Range firmware by Unwired Devices
	  www.unwds.com - info@unwds.com
	  *****************************************
	  Version: 1.10 (Mar  6 2017 17:51:59)

	  [config] Configuration loaded from NVRAM
	  [node] Initializing...
	  02:09: [!] Device is not configured yet. Type "help" to see list of possible configuration commands.
	  [!] Configure the node and type "reboot" to reboot and apply settings.
	  [ node configuration ]
	  NOJOIN = no
	  JOINKEY = 0x....1111
	  EUI64 = 0x00000000000011ff
	  APPID64 = 0x0000000000000001
	  REGION = Europe 868[03]
	  CHANNEL = 0 [868100000]
	  DATARATE = 0
	  CLASS = A
	  MAXRETR = 0
	  [ enabled modules ]
	  <no modules enabled>
	  > set region 1
	  02:22: set region 1
	  > set class C
	  02:28: set class C
	  > save
	  02:29: save
	  [*] Saving configuration...
	  [done] Configuration saved. Type "reboot" to apply changes.
	  > reboot
	  02:33: reboot
	  яmain(): This is RIOT! (Version: v1.9-1713-gc050-Alex-lora-star-simple)
	  *****************************************
	  Unwired Range firmware by Unwired Devices
	  www.unwds.com - info@unwds.com
	  *****************************************
	  Version: 1.10 (Mar  6 2017 17:51:59)

	  [config] Configuration loaded from NVRAM
	  [node] Initializing...
	  [ node configuration ]
	  NOJOIN = no
	  JOINKEY = 0x....1111
	  EUI64 = 0x00000000000011ff
	  APPID64 = 0x0000000000000001
	  REGION = Russia 868[02]
	  CHANNEL = 0 [868850000]
	  DATARATE = 0
	  CLASS = C
	  MAXRETR = 0
	  [ enabled modules ]
	  <no modules enabled>
	  init_radio: sx1276 initialization done
	  ls_init: creating uplink frame queue handler thread...
	  ls: uplink frame queue handler thread started
	  ls_init: creating timeouts handler thread...
	  02:34: > 
	  02:37: ls: first RX window expired
	  >mhdr=0xFF, mic=0x9BA1EF, addr=0xFFFFFFFF, <JOIN_REQ> fid=0x00 (61 bytes) [0 left]
	  02:40: sx1276: transmission done.
	  02:43: ls: first RX window expired
	  ls: staying in RX mode
	  02:50: ls-ed: join request expired
	  ls: join request timed out, resending
	  ls-ed: random delay 5 s
	  02:55: ls-ed: rejoining
	  02:58: ls: first RX window expired
	  >mhdr=0xFF, mic=0x3AB7C7, addr=0xFFFFFFFF, <JOIN_REQ> fid=0x00 (61 bytes) [0 left]
	  03:01: sx1276: transmission done.
	  03:04: ls: first RX window expired
	  ls: staying in RX mode
	  lsmod
	  lsmod
	  [ available modules ]
	  [-] gpio (id: 1)
	  [-] 4btn (id: 2)
	  [-] gps (id: 3)
	  [-] lsm6ds3 (id: 4)
	  [-] lm75 (id: 5)
	  [-] lmt01 (id: 6)
	  [-] uart (id: 7)
	  [-] sht21 (id: 8)
	  [-] pir (id: 9)
	  03:05: [-] 6adc (id: 10)
	  [-] lps331 (id: 11)
	  [-] 4counter (id: 12)
	  [-] echo (id: 13)
	  [-] pwm (id: 14)
	  [-] opt3001 (id: 15)
	  [-] dali (id: 16)
	  [-] bme280 (id: 17)
	  > 
	  03:11: ls-ed: join request expired
	  ls: join request timed out, resending
	  ls-ed: random delay 40 s
	  03:22: mod 5 1
	  mod 5 1
	  mod: lm75 [5] enabled. Save and reboot to apply changes
	  > mod 5 1
	  03:26: mod 5 1
	  mod: lm75 [5] enabled. Save and reboot to apply changes
	  > mod
	  03:27: mod
	  Usage: mod <modid> <0|1>. Example: mod 7 1
	  > save
	  03:31: save
	  [*] Saving configuration...
	  [done] Configuration saved. Type "reboot" to apply changes.
	  > reboot
	  03:33: reboot
	  яmain(): This is RIOT! (Version: v1.9-1713-gc050-Alex-lora-star-simple)
	  *****************************************
	  Unwired Range firmware by Unwired Devices
	  www.unwds.com - info@unwds.com
	  *****************************************
	  Version: 1.10 (Mar  6 2017 17:51:59)

	  [config] Configuration loaded from NVRAM
	  [node] Initializing...
	  [ node configuration ]
	  NOJOIN = no
	  JOINKEY = 0x....1111
	  EUI64 = 0x00000000000011ff
	  APPID64 = 0x0000000000000001
	  REGION = Russia 868[02]
	  CHANNEL = 0 [868850000]
	  DATARATE = 0
	  CLASS = C
	  MAXRETR = 0
	  [ enabled modules ]
	  [+] lm75 (id: 5)
	  init_radio: sx1276 initialization done
	  ls_init: creating uplink frame queue handler thread...
	  ls: uplink frame queue handler thread started
	  ls_init: creating timeouts handler thread...
	  [unwds] initializing "lm75" module...
	  lm75 shell command added
	  03:34: > 
	  03:37: ls: first RX window expired
	  >mhdr=0xFF, mic=0x39710B, addr=0xFFFFFFFF, <JOIN_REQ> fid=0x00 (61 bytes) [0 left]
	  03:40: sx1276: transmission done.
	  03:43: lm75
	  lm75
	  lm75 get - get results now
	  > ls: first RX window expired
	  ls: staying in RX mode
	  03:50: ls-ed: join request expired
	  ls: join request timed out, resending
	  ls-ed: random delay 20 s
	  03:52: lm75 get
	  lm75 get
	  [umdk-lm75] Temperature: 0.0 C
	  > lsmod
	  04:00: lsmod
	  [ available modules ]
	  [-] gpio (id: 1)
	  [-] 4btn (id: 2)
	  [-] gps (id: 3)
	  [-] lsm6ds3 (id: 4)
	  [+] lm75 (id: 5)
	  [-] lmt01 (id: 6)
	  [-] uart (id: 7)
	  [-] sht21 (id: 8)
	  [-] pir (id: 9)
	  [-] 6adc (id: 10)
	  [-] lps331 (id: 11)
	  [-] 4counter (id: 12)
	  [-] echo (id: 13)
	  [-] pwm (id: 14)
	  [-] opt3001 (id: 15)
	  [-] dali (id: 16)
	  [-] bme280 (id: 17)
	  > mod 15 1
	  04:02: mod 15 1
	  mod: opt3001 [15] enabled. Save and reboot to apply changes
	  > save
	  04:04: save
	  [*] Saving configuration...
	  [done] Configuration saved. Type "reboot" to apply changes.
	  > reboot
	  04:06: reboot
	  яmain(): This is RIOT! (Version: v1.9-1713-gc050-Alex-lora-star-simple)
	  *****************************************
	  Unwired Range firmware by Unwired Devices
	  www.unwds.com - info@unwds.com
	  *****************************************
	  Version: 1.10 (Mar  6 2017 17:51:59)

	  [config] Configuration loaded from NVRAM
	  [node] Initializing...
	  [ node configuration ]
	  NOJOIN = no
	  JOINKEY = 0x....1111
	  EUI64 = 0x00000000000011ff
	  APPID64 = 0x0000000000000001
	  REGION = Russia 868[02]
	  CHANNEL = 0 [868850000]
	  DATARATE = 0
	  CLASS = C
	  MAXRETR = 0
	  [ enabled modules ]
	  [+] lm75 (id: 5)
	  [+] opt3001 (id: 15)
	  init_radio: sx1276 initialization done
	  ls_init: creating uplink frame queue handler thread...
	  ls: uplink frame queue handler thread started
	  ls_init: creating timeouts handler thread...
	  [unwds] initializing "lm75" module...
	  lm75 shell command added
	  [unwds] initializing "opt3001" module...
	  [opt3001] Publish period: 1 min
	  [opt3001] Initializing opt3001 on I2C #1
	  [opt3001 driver] Error: sensor not found
	  [umdk-opt3001] Unable to init sensor!
	  04:08: > 
	  04:11: ls: first RX window expired
	  >mhdr=0xFF, mic=0x39B622, addr=0xFFFFFFFF, <JOIN_REQ> fid=0x00 (61 bytes) [0 left]
	  04:12: opt3001
	  opt3001
	  shell: command not found: opt3001
	  > 
	  04:13: sx1276: transmission done.
	  04:17: ls: first RX window expired
	  ls: staying in RX mode
	  04:24: ls-ed: join request expired
	  ls: join request timed out, resending
	  ls-ed: random delay 6 s
	  04:30: ls-ed: rejoining
	  04:33: ls: first RX window expired
	  >mhdr=0xFF, mic=0x3B73D0, addr=0xFFFFFFFF, <JOIN_REQ> fid=0x00 (61 bytes) [0 left]
	  04:35: sx1276: transmission done.
	  04:38: ls: first RX window expired
	  ls: staying in RX mode
	  04:45: ls-ed: join request expired
	  ls: join request timed out, resending
	  ls-ed: random delay 36 s
	  05:21: ls-ed: rejoining
	  05:24: ls: first RX window expired
	  >mhdr=0xFF, mic=0x4F0512, addr=0xFFFFFFFF, <JOIN_REQ> fid=0x00 (61 bytes) [0 left]
	  05:26: sx1276: transmission done.
	  05:29: ls: first RX window expired
	  ls: staying in RX mode
	  05:36: ls-ed: join request expired
	  ls: join request timed out, resending
	  ls-ed: random delay 66 s
	  06:42: ls-ed: rejoining
	  06:45: ls: first RX window expired
	  >mhdr=0xFF, mic=0xB385BE, addr=0xFFFFFFFF, <JOIN_REQ> fid=0x00 (61 bytes) [0 left]
	  06:48: sx1276: transmission done.
	  06:51: ls: first RX window expired
	  ls: staying in RX mode
	  06:58: ls-ed: join request expired
	  ls: join request timed out, resending
	  ls-ed: random delay 102 s

