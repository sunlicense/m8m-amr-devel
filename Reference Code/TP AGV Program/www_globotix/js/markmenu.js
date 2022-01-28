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

var oldURL;
var NGZ;

var PM = new ROSLIB.Param({
    ros : ros,
    name : 'mapflag'
});


function markmenustart() {
	document.getElementById("linearRange").oninput();
	document.getElementById("angularRange").oninput();
	cmdreq.cmd = 91;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");
		} 
  });
	oldURL = document.referrer;
	NGZ = false;
}

function markNGZ() {
	if (!NGZ) {
		document.getElementById("startpos").disabled = true;
		document.getElementById("endpos").disabled = true;
		document.getElementById("home").disabled = true;
		document.getElementById("dockpos").disabled = true;
		document.getElementById("savepos").disabled = true;
		document.getElementById("resetposIMG").src="imgs/newposdisable.png";
		document.getElementById("resetpos").disabled = true;		
		NGZ = true;
	} else {
		document.getElementById("startpos").disabled = false;
		document.getElementById("endpos").disabled = false;
		document.getElementById("home").disabled = false;
		document.getElementById("dockpos").disabled = false;
		document.getElementById("savepos").disabled = false;
		document.getElementById("resetposIMG").src="imgs/newpos.png";
		document.getElementById("resetpos").disabled = false;		
		NGZ = false;
	}
}

function newPOS() {
	NGZ = false;
	document.getElementById("startpos").disabled = false;
	document.getElementById("endpos").disabled = false;
	document.getElementById("home").disabled = false;
	document.getElementById("dockpos").disabled = false;
	document.getElementById("savepos").disabled = false;
	document.getElementById("resetpos").disabled = false;
	cmdreq.cmd = 99;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");
		} 
  });
}

function markSP() {
	cmdreq.cmd = 91;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");
		} 
  });
}

function markEP() {
}

function markDock() {
}

function savePos() {
}

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





