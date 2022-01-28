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


function openKB() {
	document.getElementById("pass").focus();
}


function login() {
	var passwd = document.getElementById("pass").value;
	cmdreq.cmd = 49;	
	cmdreq.lps = passwd;
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == 1) {
			document.getElementById("status").innerHTML = "Status : Login Successful" ;	
			Keyclose();
			PM.name = 'login_status';
			PM.set(true);
			location.replace(oldURL);	
		} else {
			document.getElementById("status").innerHTML = "Status : Wrong Password ! Try Again" ;	
		}
  });
}

function LoginScreen() {
	// Create main elements
  var mainElem = document.createElement("div");
  var tableElem = document.createElement("table");
	tableElem.createCaption().innerHTML = "<h1>RAC AGV Login</h1>";
	var tr = tableElem.insertRow(0);
	var tc = tr.insertCell(0);
	tc.colSpan = "3";
	tc.innerHTML = "<h1>Password</h1>";	
  mainElem.classList.add("center-login");
	mainElem.style.zIndex = "1";
	mainElem.appendChild(tableElem);
	document.body.appendChild(mainElem);
}


