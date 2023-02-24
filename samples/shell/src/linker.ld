__stack_size = 4K;
ENTRY(_start)

SECTIONS
{
	. = 0x80000000;
	.text : ALIGN(16) {
		*(.text)
	}

	.data : ALIGN(16) {
		*(.rodata)
		*(.rodata.*)
		*(.data)
		*(.data.*)
	}

	.bss : ALIGN( 16 ) {
		*(.bss)
		*(.bss.*)
	}

	.stack : ALIGN( 16 ) {
		_estack = .;
	}

    _sstack = _estack + __stack_size; 
}

