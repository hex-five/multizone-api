# multizone-api

Multi Zone API v2

The RISC-V multi zone API definition is free as in “free speech” and free as in “free lunch”. The API definition is a free and open standard completely open sourced under ISC permissive license for “any use”. It is maintained by [Hex Five Security, Inc.](http://hex-five.com) and we welcome everyone’s direct contributions in the form of GitHub pull requests. Everyone can write their own implementation of the free and open API. We are strong supporter of the commercial open source movement and we actively promote the development of a global vibrant community! 

The MultiZone™ Security implementation provided by Hex Five includes MultiZone™ Configurator, MultiZone™ nanoKernel, and MultiZone™ Interzone Communications. These components are distributed as part of the [MultiZone™ Security SDK](https://github.com/hex-five/multizone-sdk), free of charge for evaluation only. Commercial use is restricted and requires Hex Five commercial license, sometime included with the hardware itself, but never ever royalties – as we hate royalties as much as you do! Hex Five’s core components are suitable for formal verification and everyone is welcome to perform formal verification and to publish models and results. In addition, Hex Five’s source code is always available to commercial licensees for inspection and detailed code review in a clean room environment.

For more information see http://hex-five.com/faq

<br>

|Function |	Syntax and Function |	Example |
|---------|---------------------|---------|
|ECALL_YIELD|`void ECALL_YIELD()`<br> Indicates to the scheduler that the active zone has nothing pressing to do and causes the kernel to switch context to the next zone.| `ECALL_YIELD();`<br>In the case of a three zone implementation with a tick time of 10ms, the maximum time to come back to context is 20ms, faster if the other zones Yield as well.|
|ECALL_SEND|`int ECALL_SEND(zone, msg)`<br> Send transmits a message from the current zone to the zone; message size is always 16-byte long. The value returned is 1 if the message has been sent, otehrwise 0 indicates destination inbox full.|`int state = ECALL_SEND(1, {201, 0, 0 ,0});`<br>Sends 4 x 4-byte integers to Zone 1; state = 1 if successful transmission.|
|ECALL_RECV|`int ECALL_RECV(zone, msg)`<br> Receive reads a message from the zone inbox and copies it to the memory pointed by msg; message size is always 64 bytes. The value returned is 1 if a new message has been received, otherwise 0 indicates no new message.|`int msg[4]={0,0,0,0};`<br>`int state = ECALL_RECV(1, msg);`<br>If a new message exists in the mailbox from zone 1, state = 1 and it copies 4 4-byte integers to the array msg, otherwise state = 0.|
|ECALL_CSRR|`unsigned long ECALL_CSRR(csr)`<br> Return the value of the csr. This is an optional high-speed replacement for the built-in trap & emulation access to privileged registers. Traps into Invalid Instruction if the csr is not in this list:<br>| TBD
|| 0 CSR_MSTATUS
|| 1 CSR_MIE
|| 2 CSR_MTVEC
|| 3 CSR_MSCRATCH
|| 4 CSR_MEPC
|| 5 CSR_MCAUSE
|| 6 CSR_MTVAL
|| 7 CSR_MIP
||
|| 8 CSR_MISA
|| 9 CSR_MVENDORID
||10 CSR_MARCHID
||11 CSR_MIMPID
||12 CSR_MHARTID
||
||13 CSR_MCYCLE
||14 CSR_MINSTRET
||15 CSR_MHPMCOUNTER3
||16 CSR_MHPMCOUNTER4
||
||17 CSR_MCYCLEH
||18 CSR_MINSTRETH
||19 CSR_MHPMCOUNTER3H
||20 CSR_MHPMCOUNTER4H


