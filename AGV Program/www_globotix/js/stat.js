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

var statlistener = new ROSLIB.Topic({
    ros : ros,
    name : '/tstat',
    messageType : 'htbot/stat'
  });

statlistener.subscribe(function(message) {
	document.getElementById("voltage").innerHTML = "<img src='imgs/motor.png' width='25px' height='18px' /> : "+message.avgvolt.toFixed(2)+"V" ;
	document.getElementById("current").innerHTML = "<img src='imgs/motor.png' width='25px' height='18px' /> : "+message.avgcurr.toFixed(2)+"A" ;
	document.getElementById("batlevel").innerHTML = "<img src='imgs/battery.jpg' width='25px' height='18px' /> : "+message.batlevel.toFixed(2)+"%";
	document.getElementById("time").innerHTML = message.stime ;	
	document.getElementById("speed").innerHTML = "<img src='imgs/speed.png' width='25px' height='19px' /> : "+message.curspeed.toFixed(2)+"m/s";
	if (message.laser2d == 1) {
		document.getElementById("laser2d").innerHTML = "2D<img src='imgs/laser.png' width='25px' height='19px' /><i style='font-size:18px;color:#00ff00' class='material-icons'>done</i>";
	} else {
		document.getElementById("laser2d").innerHTML = "2D<img src='imgs/laser.png' width='25px' height='19px' /><i style='font-size:18px;color:#ff0000' class='material-icons'>cancel</i>";
	}
	if (message.laser3d == 1) {
		document.getElementById("laser3d").innerHTML = "3D<img src='imgs/laser.png' width='25px' height='19px' /><i style='font-size:18px;color:#00ff00' class='material-icons'>done</i>";
	} else {
		document.getElementById("laser3d").innerHTML = "3D<img src='imgs/laser.png' width='25px' height='19px' /><i style='font-size:18px;color:#ff0000' class='material-icons'>cancel</i>";
	}
	if (message.cam3d == 1) {
		document.getElementById("cam3d").innerHTML = "<img src='imgs/cam3D.png' width='25px' height='19px' /><i style='font-size:18px;color:#00ff00' class='material-icons'>done</i>";
	} else {
		document.getElementById("cam3d").innerHTML = "<img src='imgs/cam3D.png' width='25px' height='19px' /><i style='font-size:18px;color:#ff0000' class='material-icons'>cancel</i>";
	}
});



