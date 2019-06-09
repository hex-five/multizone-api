# multizone-api
# Copyright(C) 2019 Hex Five Security, Inc.
Multi Zone API v1.1.1

The design point of the MultiZone nanoKernel is to be minimalist - additional services can be built into Zones as needed.

Whats new in V1.1.0
* ECALL_SEND now provides backpressure.  
   In v0.1.1 - void ECALL_SEND would over-write the destination whether or not the mailbox had been read or not.
   
   In v1.1.0 - int ECALL_SEND returns a 1 if it successfully writes a value to an empty mailbox (ie a mailbox without an unread message) or a 0 if it fails to write to a mailbox with an unread message.  
   
   Mailbox read state is tracked by the nanoKernel
   
* ECALL_READ now provides an indication of whether the message read is new or now
   In v0.1.1 void ECALL_READ provided no secondary information on whether the message was new or not.
   
   In v1.1.0 int ECALL_READ returns a 1 if there is a new message was read and a 0 if no new mesasge was read.

v 1.1.0 API definition is:

|Function |	Syntax and Function |	Example |
|---------|---------------------|---------|
|ECALL_YIELD|`void ECALL_YIELD();`<br> Indicates to the nanoKernel scheduler that the Zone has nothing pressing to do and causes the nanoKernel to immediately move to the next Zone in context.| `ECALL_YIELD();`<br>In the case of a three zone implementation with a tick time of 10ms, the maximum time to come back to context is 20ms, faster if the other zones Yield as well.|
|ECALL_SEND|`int ECALL_SEND([Zone #], [0-3][Int]);`<br> Send transmits a message from the current zone to the [Zone #]; the message size is an array of [4] integers and the nanoKernel manages transmission with no shared memory.  The value returned is 1 if the receiving mailbox is empty and transmission successful or 0 if the receiving mailbox is full and transmission was blocked.|`int state = ECALL_SEND(1, {201, 0, 0 ,0});`<br>Sends an array to Zone 1 of {201, 0, 0, 0}; state = 1 if successful transmission.|
|ECALL_RECV|`int ECALL_RECV[Zone #], [0-3][int]);`<br>Checks the mailbox of the current Zone for a message from the listed Zone #, if there is a new message the it returns 1, if no new messaage it returns 0.  The value of the message is copied into the the array structure provided.|	`int msg[4]={0,0,0,0};`<br>`int state = ECALL_RECV(1, msg);`<br>If a newmessage exists in the mailbox from zone 1, state = 1 and it copies it to msg, otherwise state = 0.|
|ECALL_CSRS_MIE|`void ECALL_CSRS_MIE();`<br>Secure user-mode emulation of the Machine Status Register (mstatus) MIE bit. Enables all interrupts (PLIC + CLINT) mapped to the zone including the soft timer (trap 0x3). The operation is atomic with respect to the context of the zone.| `ECALL_CSRS_MIE();`|
|ECALL_CSRC_MIE|`void ECALL_CSRC_MIE();`<br>Secure user-mode emulation of the Machine Status Register (mstatus) MIE bit. Disables all interrupts (PLIC + CLINT) mapped to the zone including the soft timer (trap 0x3). The operation is atomic with respect to the context of the zone.| `ECALL_CSRC_MIE();`|
|ECALL_TRP_VECT	|`void ECALL_TRP_VECT([Exception Code], [Trap Handler])`<br>Registers a handler against a trap generated by anauthorized instructions; the TRAP #s are defined in the RISC-V Privileged Architectures definition V1.1, Table 3.6 Interrupt 0 types. https://riscv.org/specifications/privileged-isa/ |`ECALL_TRP_VECT(0x0, trap_0x0_handler);`<br>Where trap_0x0_handler is registered at the User level of privilege with:<br>`Void trap_0x0_handler(void)__attribute__((interrupt("user")));`<br>`void trap_0x0_handler(void){`<br>`     // Your handler code here`<br>`}`|
|ECALL_IRQ_VECT	|`void ECALL_IRQ_VECT([Interrupt #], [Trap Handler])`<br>Registers a handler for an interrupt that has been assigned to a Zone in the multizone.cfg file. <br> When an interrupt occurs, the nanoKernel will immediately pull the zone assigned to that interrupt into context and execute the registered interrupt handler.	|`ECALL_IRQ_VECT(11, button_0_handler);`<br>Where button_0_handler is a registered at the user level of privilege with:<br>`void button_1_handler(void)__attribute__((interrupt("user")));`<br>`void button_1_handler(void){`<br>`  // interrupt handler here`<br>`}`|
|ECALL_CSRW_MTIMECMP	|`void ECALL_CSRW_MTIMECMP(uint64_t)`<br>Secure user-mode emulation of the machine-mode timer compare register (mtimecmp). Causes a trap 0x3 exception when the mtime register contains a value greater than or equal to the value assigned. Each zone has its own secure instance of timer and trap handler. Per RISC-V specs this is a one-shot timer: once set it will execute its callback function only once. Note that mtime and mtimecmp size is 64-bit even on rv32 architecture. Registering the trap 0x3 handler sets the value of mtimecmp to zero to prevent spurious interrupts. If the timer is set but no handler is registered the exception is ignored.	| `#include <libhexfive.h>` <br> `...` <br> <br> `void trap_0x3_handler(void)__attribute__((interrupt("user")));` <br> `void trap_0x3_handler(void){` <br>   `// do something `<br>`	// restart the timer` <br>`	uint64_t T = 10; // ms `<br>`	uint64_t T0 = ECALL_CSRR_MTIME();` <br>	`uint64_t T1 = T0 + T*32768/1000; `<br>` 	ECALL_CSRR_MTIMECMP(T1); `<br> <br> `} `<br> `...` <br> `main () { `<br> `ECALL_TRP_VECT(0x3, trap_0x3_handler); // register 0x3 Soft timer `<br>	`while(1){` <br> `	// do many things `<br>	`} `<br>` } `<br>
|ECALL_CSRR_MTIME()|`Int64 ECALL_CSRR_MTIME()`<br>	Returns MTIME to a variable in a zone, MTIME is a privileged registered normally only available in M mode.	|`Int64 mtime = ECALL_CSRR_MTIME();`|
|ECALL_CSRR_MCYCLE()|`Int64 ECALL_CSRR_MCYCLE()`<br>	Returns MCYCLE to a variable in a zone, MCYCLE is a privileged registered normally only available in M mode.	|`Int64 mcycle = ECALL_CSRR_MCYCLE();`
|ECALL_CSRR_MINSTR()|`Int64 ECALL_CSRR_MINSTR()`<br>	Returns MINSTR to a variable in a zone, MINSTR is a privileged registered normally only available in M mode.	|`Int64 minstr = ECALL_CSRR_MINSTR();`
|ECALL_CSRR_MHPMC3()|`Int64 ECALL_CSRR_MHPMC3()`<br>	Returns MHPMC3 to a variable in a zone, MHPMC3 is a privileged registered normally only available in M mode.	|`Int64 mhpmc3 = ECALL_CSRR_MHPMC3();`
|ECALL_CSRR_MHPMC4()|`Int64 ECALL_CSRR_MHPMC4()`<br>	Returns MHPMC4 to a variable in a zone, MHPMC4 is a privileged registered normally only available in M mode.	|`Int64 mhpmc3 = ECALL_CSRR_MHPMC4();`
|ECALL_CSRR_MISA()|`Int64 ECALL_CSRR_MISA()`<br>	Returns MISA to a variable in a zone, MISA is a privileged registered normally only available in M mode.	|`Int64 misa = ECALL_CSRR_MISA();`
|ECALL_CSRR_MVENDID()|`Int64 ECALL_CSRR_MVENDID()`<br>	Returns MVENDID to a variable in a zone, MVENDID is a privileged registered normally only available in M mode.	|`Int64 misa = ECALL_CSRR_MVENDID();`
|ECALL_CSRR_MARCHID()|`Int64 ECALL_CSRR_MARCHID()`<br>	Returns MARCHID to a variable in a zone, MARCHID is a privileged registered normally only available in M mode.	|`Int64 marchid = ECALL_CSRR_MARCHID();`
|ECALL_CSRR_MIMPID()|`Int64 ECALL_CSRR_MIMPID()`<br>	Returns MIMPID to a variable in a zone,  MIMPID  is a privileged registered normally only available in M mode.	|`Int64 mimpid = ECALL_CSRR_ MIMPID ();`
|ECALL_CSRR_MHARTID()|`Int64 ECALL_CSRR_MHARTID()`<br>	Returns MHARTID to a variable in a zone,  MHARTID is a privileged registered normally only available in M mode.	|`Int64 mhardid = ECALL_CSRR_ MHARTID ();`

v 0.1.1 API definition was:

|Function |	Syntax and Function |	Example |
|---------|---------------------|---------|
|ECALL_YIELD|`void ECALL_YIELD();`<br> Indicates to the nanoKernel scheduler that the Zone has nothing pressing to do and causes the nanoKernel to immediately move to the next Zone in context.| `ECALL_YIELD();`<br>In the case of a three zone implementation with a tick time of 10ms, the maximum time to come back to context is 20ms, faster if the other zones Yield as well.|
|ECALL_SEND|`void ECALL_SEND([Zone #], [0-3][Int]);`<br> Send transmits a message from the current zone to the [Zone #]; the message size is an array of [4] integers and the nanoKernel manages transmission with no shared memory.|`ECALL_SEND(1, {201, 0, 0 ,0});`<br>Sends an array to Zone 1 of {201, 0, 0, 0}|
|ECALL_RECV|`void ECALL_RECV[Zone #], [0-3][int]);`<br>Checks the mailbox of the current Zone for a message from the listed Zone #, if a message exists it copies it to the array structure provided.|	`int msg[4]={0,0,0,0};`<br>`ECALL_RECV(1, msg);`<br>If a message exists in the mailbox from zone 1, it copies it to msg, otherwise msg value is unchanged.|
|ECALL_CSRS_MIE|`void ECALL_CSRS_MIE();`<br>Secure user-mode emulation of the Machine Status Register (mstatus) MIE bit. Enables all interrupts (PLIC + CLINT) mapped to the zone including the soft timer (trap 0x3). The operation is atomic with respect to the context of the zone.| `ECALL_CSRS_MIE();`|
|ECALL_CSRC_MIE|`void ECALL_CSRC_MIE();`<br>Secure user-mode emulation of the Machine Status Register (mstatus) MIE bit. Disables all interrupts (PLIC + CLINT) mapped to the zone including the soft timer (trap 0x3). The operation is atomic with respect to the context of the zone.| `ECALL_CSRC_MIE();`|
|ECALL_TRP_VECT	|`void ECALL_TRP_VECT([Exception Code], [Trap Handler])`<br>Registers a handler against a trap generated by anauthorized instructions; the TRAP #s are defined in the RISC-V Privileged Architectures definition V1.1, Table 3.6 Interrupt 0 types. https://riscv.org/specifications/privileged-isa/ |`ECALL_TRP_VECT(0x0, trap_0x0_handler);`<br>Where trap_0x0_handler is registered at the User level of privilege with:<br>`Void trap_0x0_handler(void)__attribute__((interrupt("user")));`<br>`void trap_0x0_handler(void){`<br>`     // Your handler code here`<br>`}`|
|ECALL_IRQ_VECT	|`void ECALL_IRQ_VECT([Interrupt #], [Trap Handler])`<br>Registers a handler for an interrupt that has been assigned to a Zone in the multizone.cfg file. <br> When an interrupt occurs, the nanoKernel will immediately pull the zone assigned to that interrupt into context and execute the registered interrupt handler.	|`ECALL_IRQ_VECT(11, button_0_handler);`<br>Where button_0_handler is a registered at the user level of privilege with:<br>`void button_1_handler(void)__attribute__((interrupt("user")));`<br>`void button_1_handler(void){`<br>`  // interrupt handler here`<br>`}`|
|ECALL_CSRW_MTIMECMP	|`void ECALL_CSRW_MTIMECMP(uint64_t)`<br>Secure user-mode emulation of the machine-mode timer compare register (mtimecmp). Causes a trap 0x3 exception when the mtime register contains a value greater than or equal to the value assigned. Each zone has its own secure instance of timer and trap handler. Per RISC-V specs this is a one-shot timer: once set it will execute its callback function only once. Note that mtime and mtimecmp size is 64-bit even on rv32 architecture. Registering the trap 0x3 handler sets the value of mtimecmp to zero to prevent spurious interrupts. If the timer is set but no handler is registered the exception is ignored.	| `#include <libhexfive.h>` <br> `...` <br> <br> `void trap_0x3_handler(void)__attribute__((interrupt("user")));` <br> `void trap_0x3_handler(void){` <br>   `// do something `<br>`	// restart the timer` <br>`	uint64_t T = 10; // ms `<br>`	uint64_t T0 = ECALL_CSRR_MTIME();` <br>	`uint64_t T1 = T0 + T*32768/1000; `<br>` 	ECALL_CSRR_MTIMECMP(T1); `<br> <br> `} `<br> `...` <br> `main () { `<br> `ECALL_TRP_VECT(0x3, trap_0x3_handler); // register 0x3 Soft timer `<br>	`while(1){` <br> `	// do many things `<br>	`} `<br>` } `<br>
|ECALL_CSRR_MTIME()|`Int64 ECALL_CSRR_MTIME()`<br>	Returns MTIME to a variable in a zone, MTIME is a privileged registered normally only available in M mode.	|`Int64 mtime = ECALL_CSRR_MTIME();`|
|ECALL_CSRR_MCYCLE()|`Int64 ECALL_CSRR_MCYCLE()`<br>	Returns MCYCLE to a variable in a zone, MCYCLE is a privileged registered normally only available in M mode.	|`Int64 mcycle = ECALL_CSRR_MCYCLE();`
|ECALL_CSRR_MINSTR()|`Int64 ECALL_CSRR_MINSTR()`<br>	Returns MINSTR to a variable in a zone, MINSTR is a privileged registered normally only available in M mode.	|`Int64 minstr = ECALL_CSRR_MINSTR();`
|ECALL_CSRR_MHPMC3()|`Int64 ECALL_CSRR_MHPMC3()`<br>	Returns MHPMC3 to a variable in a zone, MHPMC3 is a privileged registered normally only available in M mode.	|`Int64 mhpmc3 = ECALL_CSRR_MHPMC3();`
|ECALL_CSRR_MHPMC4()|`Int64 ECALL_CSRR_MHPMC4()`<br>	Returns MHPMC4 to a variable in a zone, MHPMC4 is a privileged registered normally only available in M mode.	|`Int64 mhpmc3 = ECALL_CSRR_MHPMC4();`
|ECALL_CSRR_MISA()|`Int64 ECALL_CSRR_MISA()`<br>	Returns MISA to a variable in a zone, MISA is a privileged registered normally only available in M mode.	|`Int64 misa = ECALL_CSRR_MISA();`
|ECALL_CSRR_MVENDID()|`Int64 ECALL_CSRR_MVENDID()`<br>	Returns MVENDID to a variable in a zone, MVENDID is a privileged registered normally only available in M mode.	|`Int64 misa = ECALL_CSRR_MVENDID();`
|ECALL_CSRR_MARCHID()|`Int64 ECALL_CSRR_MARCHID()`<br>	Returns MARCHID to a variable in a zone, MARCHID is a privileged registered normally only available in M mode.	|`Int64 marchid = ECALL_CSRR_MARCHID();`
|ECALL_CSRR_MIMPID()|`Int64 ECALL_CSRR_MIMPID()`<br>	Returns MIMPID to a variable in a zone,  MIMPID  is a privileged registered normally only available in M mode.	|`Int64 mimpid = ECALL_CSRR_ MIMPID ();`
|ECALL_CSRR_MHARTID()|`Int64 ECALL_CSRR_MHARTID()`<br>	Returns MHARTID to a variable in a zone,  MHARTID is a privileged registered normally only available in M mode.	|`Int64 mhardid = ECALL_CSRR_ MHARTID ();`
