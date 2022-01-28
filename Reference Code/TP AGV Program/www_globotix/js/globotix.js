//var hostIP = "ws://127.0.0.1:8080";
var hostIP = "ws://"+location.host+":8080";
var ros = new ROSLIB.Ros();
ros.connect(hostIP);
var cmdsrv = new ROSLIB.Service({
    ros : ros,
    name : '/web_cmd',
    serviceType : 'htbot/mqueue'
});

var checkCleaningComplete;
var pauseCleaning;
var cleaningStop;
var cleaningStatus;
var oldURL;
var timeoutid;

var PM = new ROSLIB.Param({
    ros : ros,
    name : 'mapflag'
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

function checkCleaningStatus() {	
	//console.log("---- Check Cleaning Status ---");
	PM.name = 'cleaningStatus';
	PM.get(function(value) {
		if (value == 2) {
			clearInterval(checkCleaningComplete);
			document.getElementById("mode").innerHTML = "Cleaning Completed.";	
			document.getElementById("pausestart").innerHTML = "<h2>ReClean</h2>";
			document.getElementById("pause").innerHTML = "<i style='font-size: 60px;' class='material-icons'>directions_run</i>";
			document.getElementById("pause").onclick = function() {startReClean()};
			document.getElementById("image").src="imgs/cleaning.png";
			document.getElementById("image").className="imgclean";
			document.getElementById("pause").disabled = false;
			document.getElementById("stopButton").disabled = false;
			cleaningCompleted = true;
			syslocked = false;
			document.removeEventListener("click",goPass); 
			clearTimeout(timeoutid);
		} else {
			cleaningCompleted = false;
			//document.getElementById("pause").innerHTML ="Cleaning";
		}
	});
}

function globotixStart() {
	
	cmdreq.cmd = 91;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");
		} else {
			restoreParam();
			checkCleaningComplete = setInterval(checkCleaningStatus, 2000);	
			timeoutid = setTimeout(function() {sysLock();}, 8000);
			if (!syslocked) {	
				document.getElementById("pause").disabled = false;
				document.getElementById("stopButton").disabled = false;
			} else {
				document.getElementById("pause").disabled = true;
				document.getElementById("stopButton").disabled = true;
			}
			if (pauseCleaning) {
				document.getElementById("mode").innerHTML = "Cleaning Paused"; 
				document.getElementById("pausestart").innerHTML = "<h2>Start</h2>";				
				document.getElementById("pause").innerHTML = "<i style='font-size: 60px;' class='material-icons'>directions_run</i>";	
			} else {
				document.getElementById("mode").innerHTML = "Cleaning"; 
				document.getElementById("pausestart").innerHTML = "<h2>Pause</h2>";
				document.getElementById("pause").innerHTML = "<i style='font-size: 60px;' class='material-icons'>pause</i>";
			}
		}
  });
}

function storeParam() {
	if(typeof(Storage) !== "undefined") {
		if (pauseCleaning) {
    	localStorage.setItem("pauseCleaning", "true");
		} else {
			localStorage.setItem("pauseCleaning", "false");
		}
		if (cleaningStop) {
    	localStorage.setItem("cleaningStop", "true");
		} else {
			localStorage.setItem("cleaningStop", "false");
		}
		if (cleaningCompleted) {
    	localStorage.setItem("cleaningCompleted", "true");
		} else {
			localStorage.setItem("cleaningCompleted", "false");
		}
		if (syslocked) {
    	localStorage.setItem("syslocked", "true");
		} else {
			localStorage.setItem("syslocked", "false");
		}
		
  } 
}

function pauseClean() {
	if (!pauseCleaning) {
		document.getElementById("mode").innerHTML = "Cleaning Paused"; 	
		document.getElementById("pausestart").innerHTML = "<h2>Start</h2>";				
		document.getElementById("pause").innerHTML = "<i style='font-size: 60px;' class='material-icons'>directions_run</i>";	
		pauseCleaning = true;
		PM.name = 'cleaningStatus';
		PM.set(10);
		//console.log("------Pause Cleaning----");
	} else {
		document.getElementById("mode").innerHTML = "Cleaning"; 
		document.getElementById("pausestart").innerHTML = "<h2>Pause</h2>";
		document.getElementById("pause").innerHTML = "<i style='font-size: 60px;' class='material-icons'>pause</i>";
		pauseCleaning = false;
		PM.name = 'cleaningStatus';
		PM.set(1);
		//console.log("--Cleaning Continue----");
	}
}


function startReClean() {
	if (cleaningCompleted) {	
		localStorage.clear();
		setParam();
		PM.name = 'cleaningStatus';
		PM.set(4) ;
		location.replace("ReClean.html");
	}
}

function restoreParam() {
	if(typeof(Storage) !== "undefined") {
    pauseCleaning = localStorage.getItem("pauseCleaning");
		pauseCleaning = (pauseCleaning === "true");
		cleaningStop = localStorage.getItem("cleaningStop");
		cleaningStop = (cleaningStop === "true");
		cleaningCompleted = localStorage.getItem("cleaningCompleted");
		cleaningCompleted = (cleaningCompleted === "true");
    syslocked = localStorage.getItem("syslocked");
		syslocked = (syslocked === "true");
  } 
}

function sysLock() {
	if (!cleaningCompleted) {	
		syslocked = true;
		document.getElementById("mode").innerHTML = "Screen Locked. Tap Screen to UnLock";
		document.getElementById("pause").disabled = true;
		document.getElementById("stopButton").disabled = true;
		document.addEventListener('click',goPass);
	}
}

function goPass(){
	storeParam();
	clearTimeout(timeoutid);
	location.replace("password.html");
}

function setParam() {
	if(typeof(Storage) !== "undefined") {
		localStorage.setItem("pauseCleaning", "false");
		localStorage.setItem("cleaningStop", "false");
		localStorage.setItem("cleaningCompleted", "false");
		localStorage.setItem("syslocked", "false");		
  } 
}

function goDockStation() {
	document.getElementById("mode").innerHTML = "AGV Returning to Docking Station";	
	PM.name = 'returnToDock';
	PM.set(true);
	cmdreq.cmd = 58;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
  });
	if(typeof(Storage) !== "undefined") {
    localStorage.clear();	
  } 
	setTimeout(function() {location.replace("index.html");}, 2000);
	
}

function gomainmenu() {
	clearTimeout(timeoutid);
	location.replace("mainmenu.html");
}

function stopClean() {
	document.getElementById("mode").innerHTML = "Cleaning Terminated";	
	document.getElementById("pausestart").innerHTML = "<h2>Dock</h2>";
	document.getElementById("pause").innerHTML = "<img src='imgs/dock.png' width='60px' height='60px' />";	
	//document.getElementById("pause").onclick = function(e) {goDockStation();};
	document.getElementById("pause").onclick = function() {goDockStation()};
	//document.getElementById("image").src="imgs/cleaningStop.png";
	document.getElementById("image").className="imgclean";
	document.getElementById("stopText").innerHTML = "<h2>MMenu</h2>";	
	document.getElementById("stopButton").innerHTML = "<i style='font-size: 60px;' class='material-icons'>home</i>";
	//document.getElementById("stopButton").onclick = function(e) {location.replace("mainmenu.html");};
	document.getElementById("stopButton").onclick = function() {gomainmenu()};
	document.removeEventListener("click",goPass); 
	clearTimeout(timeoutid);
	cleaningStop = true;
	PM.name = 'cleaningStatus';
	PM.set(0);
	localStorage.clear();		
}








