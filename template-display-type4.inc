#address-cells = <2>;
#size-cells = <0>;

led@{GRID},{LED0} {
	reg = <{GRID} {LED0}>;
	function = LED_FUNCTION_BLUETOOTH;
};

led@{GRID},{LED1} {
	reg = <{GRID} {LED1}>;
	function = LED_FUNCTION_LAN;
};

led@{GRID},{LED2} {
	reg = <{GRID} {LED2}>;
	function = LED_FUNCTION_WLAN;
};

led@{GRID},{LED3} {
	reg = <{GRID} {LED3}>;
	function = "spdif";
};

led@{GRID},{LED4} {
	reg = <{GRID} {LED4}>;
	function = "colon";
};

led@{GRID},{LED5} {
	reg = <{GRID} {LED5}>;
	function = "hdmi";
};

led@{GRID},{LED6} {
	reg = <{GRID} {LED6}>;
	function = "video";
};
