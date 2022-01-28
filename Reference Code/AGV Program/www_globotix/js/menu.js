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

var chkmodeTimer = function checkOpMode() {
	cmdreq.cmd = 55;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
    if ((result.status == -1) || (result.status == 0)) {
    	location.replace("index.html");
    } else {
    	if (result.status == 1) {
				location.replace("mainmenu.html");
			} else {
				if (result.status == 2) {
					location.replace("manual.html");
				} else {
					if (result.status == 3) {
						location.replace("auto.html");
					} else {
						if (result.status == 4) {
							location.replace("map.html");
						} else {
							location.replace("sync.html");
						}
					}
				}
			}
    }		
  });
}

function start() {
	cmdreq.cmd = 55;	// check login status
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

function login() {
	var passwd = document.getElementById("pass").value;
	cmdreq.cmd = 49;	
	cmdreq.lps = passwd;
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

function logout() {
	cmdreq.cmd = 58;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
	location.replace("index.html");
}

function manual() {
	location.replace("manual.html");
}

function map() {
	location.replace("map.html");
}

function auto() {
	location.replace("auto.html");
}

function sync() {
	location.replace("sync.html");
}

function nav() {
	cmdreq.cmd = 55;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status != 1) {
			alert("Please Login First");
			location.replace("index.html");
			return;
		} 
  });
	cmdreq.cmd = 84;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
}

function shutdown_nopwd() {	
	cmdreq.cmd = 55;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			alert("Please Login First");
			location.replace("index.html");
			return;
		} else {
			cmdreq.cmd = 2;	
			cmdsrv.callService(cmdreq, function(result) {
    		console.log('Result for service call on '
      		+ cmdsrv.name
      		+ ': '
      		+ result.status);
  		});
		}
  });
}
