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

var oldURL;
var locPause;
var checkLocalise;
var checkCleaningComplete;
var dblclick;
var pauseCleaning;
var cleaningStop;
var cleaningCompleted;
var syslocked;

function startlocalise() {
	cmdreq.cmd = 89;	// 
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");		
		} 
		checkLocalise = setInterval(checkLocalisation, 2000);
		PM.name = 'localisation_status';
		PM.set(1);
  });
	oldURL = document.referrer;
	locPause = false;
}

function pauseInit() {
	if (!locPause) {
		document.getElementById("pause").innerHTML = "Initialisation of Position Paused. Click <i class='material-icons'>location_searching</i> to Continue.." ;	
		document.getElementById("control").innerHTML = "<i class='material-icons-75'>location_searching</i>";	
		locPause = true;
		PM.name = 'localisation_status';
		PM.set(3);
	} else {
		document.getElementById("pause").innerHTML = "" ;	
		document.getElementById("control").innerHTML = "<i class='material-icons-75'>pan_tool</i>";	
		locPause = false;
		PM.name = 'localisation_status';
		PM.set(1);
	}
}

function goBack() {
	PM.name = 'localisation_status';
	PM.set(0);
	PM.name = 'init_localisation';
	PM.set(false);
	PM.name = 'stop_Localisation';
	PM.set(false);
	location.replace(oldURL);	
}

function setParam() {
	if(typeof(Storage) !== "undefined") {
		localStorage.setItem("pauseCleaning", "false");
		localStorage.setItem("cleaningStop", "false");
		localStorage.setItem("cleaningCompleted", "false");
		localStorage.setItem("syslocked", "false");		
  } 
}

function checkLocalisation() {
	PM.name = 'localisation_status';
	PM.get(function(value) {
		if (value == 1) {
			document.getElementById("pause").innerHTML = "Localising Start Position. Please wait...";
			PM.name = 'Localised_Ready';
			PM.get(function(value) {
				if (value) {
					//Testing only. Actual run remove cleaningStatus
					PM.name = 'cleaningStatus';
					PM.set(1);
					PM.name = 'localisation_status';
					PM.set(2);
					clearInterval(checkLocalise);
					setParam();
					location.replace("globotixClean.html");	
				} 
			});
		} else {
			if (value != 0) {
				document.getElementById("pause").innerHTML = "Initialisation of Position Paused. Click <i class='material-icons'>location_searching</i> to Continue.." ;
			}
		} 
	});		
}

function goToNav() {
	PM.name = 'Localised_Ready';
	PM.get(function(value) {
		if (value) {
			clearInterval(checkLocalise);
			PM.name = 'AGVModel';
			PM.get(function(value) {
				if (value == 1) {
					location.replace("globotixClean.html");					
				} else {
					location.replace("racNav.html");
				}
			});
			//location.replace("racNav.html");
		} else {
			document.getElementById("pause").innerHTML = "Localisation of Start Position Not Done. Please wait...";
		}
	});
}




