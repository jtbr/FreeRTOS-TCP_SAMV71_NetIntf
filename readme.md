## [SAMV71(Q21)](http://www.microchip.com/wwwproducts/en/ATSAMV71Q21) Network Interface for [KSZ8061RNB](http://www.microchip.com/wwwproducts/en/KSZ8061) Ethernet PHY

This NetworkInterface.c implementation is based on the FreeRTOS+TCP-
provided ATSAM4E microcontroller implementation. It relies upon the 
gmac and ethernet physical drivers also included here. These were based 
upon the ASF version 3.34.1 drivers and the FreeRTOS+TCP example.
The GMAC driver in particular and also the phy component were 
extensively modified to rectify deficiencies and omissions.

This network interface was tested with FreeRTOS 9.0.0 and is known not 
to work with FreeRTOS 8.2.3. Use of the BufferAllocation_2.c buffer 
management scheme is assumed. It has been tested and is working for 
basic TCP/IP. It has not been tested for DNS/LLNMR, but in theory 
they are supported, as are other protocols atop TCP.

Network Descriptors and Buffers are required by hardware to be in 
non-cached SRAM. That leaves you two options:
1) Leave D(ata) cache disabled.
2) Mark an area of SRAM as non-cached, and put the buffers/descriptors there. This allows the cache to operate on other areas of SRAM.

In order to do the latter, it's necessary to use the MPU to disable 
caching on an area of SRAM of large enough to cover descriptors and 
buffers (25kb as configured). This is most easily done by adding a 
`ram_nocache section` to the linker script:

```
MEMORY
{
  rom (rx)  : ORIGIN = 0x00400000, LENGTH = 0x00200000
  ram (rwx) : ORIGIN = 0x20400000, LENGTH = 0x00059C00
  ram_nocache (rwx): ORIGIN = 0x20459C00, LENGTH = 0x6400      /* 25kb for ethernet */
  /* ... */
}

/* ... */
SECTIONS
{
    /* ... */

	/* after heap/stack */
    .ram_nocache (NOLOAD):
    {
        . = ALIGN(8);
        _sram_nocache = .;
    } > ram_nocache

	/* ... */
}
```

and then (using ASF) enabling the MPU with a no-cache size of 25kb. 
First add the following to conf_board.h:

```c
#define CONF_BOARD_CONFIG_MPU_AT_INIT
#define MPU_HAS_NOCACHE_REGION
#define NOCACHE_SRAM_REGION_SIZE  0x6400  /* 25kb, overrides define in MODIFIED mpu.h */
```

then modify mpu.h to add (if this define is not already there, as it wasn't for me):

```c
#define INNER_OUTER_NORMAL_NOCACHE_TYPE(x)   (( 0x01 << MPU_RASR_TEX_Pos ) | ( DISABLE << MPU_RASR_C_Pos ) | ( DISABLE << MPU_RASR_B_Pos ) | ( x << MPU_RASR_S_Pos ))
```

and change the line about `NO_CACHE_SRAM_REGION_SIZE` to allow overriding the size:

```c
#if (defined MPU_HAS_NOCACHE_REGION) && !(defined NOCACHE_SRAM_REGION_SIZE)
#define NOCACHE_SRAM_REGION_SIZE            0x1000  /* 4kb */
#endif
```

That should do it.

If you instead choose to leave the DCache disabled, you'll need to remove the 
`__attribute__ ((section(".ram_nocache")))` portions of lines in gmac_raw_2.c.

---
Development of this network interface was funded by [Excellims](http://www.excellims.com/).