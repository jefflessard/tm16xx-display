#address-cells = <2>;
#size-cells = <0>;

led@{GRID},{LED2} {
	reg = <{GRID} {LED2}>;
	function = LED_FUNCTION_POWER;
};

led@{GRID},{LED3} {
	reg = <{GRID} {LED3}>;
	function = LED_FUNCTION_LAN;
};

led@{GRID},{LED4} {
	reg = <{GRID} {LED4}>;
	function = "colon";
};

led@{GRID},{LED5} {
	reg = <{GRID} {LED5}>;
	function = "wlan-lo";
};

led@{GRID},{LED6} {
	reg = <{GRID} {LED6}>;
	function = "wlan-hi";
};
