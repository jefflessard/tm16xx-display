led@{GRID0},{LED0} {
	reg = <{GRID0} {LED0}>;
	function = "apps";
};
led@{GRID1},{LED1} {
	reg = <{GRID1} {LED1}>;
	function = LED_FUNCTION_LAN;
};
led@{GRID2},{LED2} {
	reg = <{GRID2} {LED2}>;
	function = LED_FUNCTION_USB;
};
led@{GRID3},{LED3} {
	reg = <{GRID3} {LED3}>;
	function = LED_FUNCTION_SD;
};
led@{GRID4},{LED4} {
	reg = <{GRID4} {LED4}>;
	function = "colon";
};
led@{GRID5},{LED5} {
	reg = <{GRID5} {LED5}>;
	function = "wlan-lo";
};
led@{GRID6},{LED6} {
	reg = <{GRID6} {LED6}>;
	function = "wlan-hi";
};
