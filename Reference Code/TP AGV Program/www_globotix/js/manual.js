var hostIP = "ws://"+location.host+":8080";
var ros = new ROSLIB.Ros();
ros.connect(hostIP);
var cmdsrv = new ROSLIB.Service({
    ros : ros,
    name : '/web_cmd',
    serviceType : 'htbot/mqueue'
});

var cmdreq = new ROSLIB.ServiceRequest({
    cmd : 1,
    LP : 0,
    GN : 0,
    gps : "",
    lps : "POWER",
		pw : "",
    tx : 1.0,
    ty : 2.0,
    tz : 3.0,
    rx : 0.0,
		ry : 0.0,
    rz : 0.0,
    rw : 0.0,
    prd : 0.0,
		pra : 0.0,
    psd : 0.0,
    psa : 0.0,
    prd1 : 0.0,
    pra1 : 0.0,
    psd1 : 0.0,
    psa1 : 0.0,
    align : 0.0,
    autostart : 0.0 
});

var PM = new ROSLIB.Param({
    ros : ros,
    name : 'mapflag'
});

function linearValue() {
	//document.getElementById("tdstop").focus();
	//document.activeElement.blur();
	var lvalue = document.getElementById("linearRange").value;
	var avalue = document.getElementById("angularRange").value;  
	//var output = document.getElementById("demo"); 
	document.getElementById("linear").innerHTML = ((lvalue * 1.4)/100).toFixed(1)+" m/s"; 

	cmdreq.cmd = 86;	
	cmdreq.tx = ((lvalue * 1.4)/100);
	cmdreq.ty = ((avalue * 1.4)/100);
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
	//document.getElementById("tdstop").focus();
	//document.activeElement.blur();
}

function angularValue() {
	//document.getElementById("tdstop").focus();
	//document.activeElement.blur();
	var lvalue = document.getElementById("linearRange").value;
	var avalue = document.getElementById("angularRange").value;  
	//var output = document.getElementById("demo"); 
	document.getElementById("angular").innerHTML = ((avalue * 1.4)/100).toFixed(1)+" m/s"; 

	cmdreq.cmd = 86;	
	cmdreq.tx = ((lvalue * 1.4)/100);
	cmdreq.ty = ((avalue * 1.4)/100);
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
	//document.getElementById("tdstop").focus();
	//document.activeElement.blur();
}


var chkmodeTimer = function checkOpMode() {
	var rangeslider = document.getElementById("sliderRange"); 
	var output = document.getElementById("demo"); 
	output.innerHTML = rangeslider.value; 
	  
	rangeslider.oninput = function() { 
		output.innerHTML = this.value; 
	}
}

function start() {
	document.getElementById("linearRange").oninput();
	document.getElementById("angularRange").oninput();
	cmdreq.cmd = 91;	// check login status
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");	
		} 
  });
	//document.getElementById("tdstop").focus();
}

function forward() {
	var lvalue = document.getElementById("linearRange").value;
	var avalue = document.getElementById("angularRange").value;  
	cmdreq.cmd = 86;	
	cmdreq.tx = ((lvalue * 1.4)/100);
	cmdreq.ty = 99.0;
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
	//document.getElementById("tdstop").focus();
	//document.activeElement.blur();
}

function reverse() {
	var lvalue = document.getElementById("linearRange").value;
	var avalue = document.getElementById("angularRange").value;  
	cmdreq.cmd = 86;	
	cmdreq.tx = -((lvalue * 1.4)/100);
	cmdreq.ty = 99.0;
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
	//document.getElementById("tdstop").focus();
	//document.activeElement.blur();
}

function rotate_left() {
	var lvalue = document.getElementById("linearRange").value;
	var avalue = document.getElementById("angularRange").value;  
	cmdreq.cmd = 86;	
	cmdreq.tx = 99.0;
	cmdreq.ty = ((avalue * 1.4)/100);
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
	//document.getElementById("tdstop").focus();
	//document.activeElement.blur();
}

function rotate_right() {
	var lvalue = document.getElementById("linearRange").value;
	var avalue = document.getElementById("angularRange").value;  
	cmdreq.cmd = 86;	
	cmdreq.tx = 99.0;
	cmdreq.ty = -((avalue * 1.4)/100);
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
	//document.getElementById("tdstop").focus();
	//document.activeElement.blur();
}

function stop() {
	cmdreq.cmd = 86;	
	cmdreq.tx = 0.0;
	cmdreq.ty = 0.0;
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
	//document.getElementById("tdstop").focus();
	//document.activeElement.blur();
}

function home() {
	location.replace("mainmenu.html");
}





