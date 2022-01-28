//var hostIP = "ws://127.0.0.1:8080";
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

function indexstart() {
	var d = new Date();
	var off = d.getTimezoneOffset();
  var hr = d.getUTCHours();
	var mn = d.getUTCMinutes();
	var tt;
	hr = hr -off/60;
	if (hr < 12) {
		tt = hr+":"+mn+"AM";
	} else {
		if (hr > 12) {
			tt = (hr-12)+":"+mn+"PM";
		} else {
			tt = hr+":"+mn+"PM";
		}
	}
  //document.getElementById("time").innerHTML = tt;
	//document.getElementById("batlevel").innerHTML = "<img src='imgs/battery.jpg' width='25px' height='18px' /> : 70%";
	//document.getElementById("voltage").innerHTML = "<img src='imgs/motor.png' width='25px' height='18px' /> : 24.5V";
	//document.getElementById("speed").innerHTML = "<img src='imgs/speed.png' width='25px' height='19px' /> : 0.7m/s";
	//document.getElementById("current").innerHTML = "<img src='imgs/motor.png' width='25px' height='18px' /> : 3.5A";
	//document.getElementById("cam3d").innerHTML = "3DCam<i style='font-size:18px;color:#00ff00' class='material-icons'>done</i>";
	//document.getElementById("laser2d").innerHTML = "2DLaser<i style='font-size:18px;color:#00ff00' class='material-icons'>done</i>";
	//document.getElementById("laser3d").innerHTML = "3DLaser<i style='font-size:18px;color:#00ff00' class='material-icons'>done</i>";
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
			location.replace("mainmenu.html");					
		} else {
			document.getElementById("status").innerHTML = "Status : Wrong Password ! Try Again" ;	
		}
  });
}

function openKB() {
	document.getElementById("pass").focus();
}




