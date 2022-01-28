var hostIP = "ws://127.0.0.1:8080";
var ros = new ROSLIB.Ros();
ros.connect(hostIP);
var cmdsrv = new ROSLIB.Service({
    ros : ros,
    name : '/web_cmd',
    serviceType : 'htbot/mqueue'
});

var numplan;
var numLP;
var planselectidx;
var oldURL;
var locPause;

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

function home() {
	location.replace("mainmenu.html");	
}

function returntoauto() {
	location.replace("auto.html");	
}

function setting() {
	location.replace("setting.html");	
}

function goLocatiion() {
	var myImg = document.querySelector("#racnav");// mapimg
	var currWidth = myImg.clientWidth;
	var currHeight = myImg.clientHeight;
	alert("Current width=" + currWidth + ", " + "Original height=" + currHeight);
}

function startpoint() {	
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
	oldURL = document.referrer;
	locPause = false;
}

function startnav() {	
	cmdreq.cmd = 55;	// check login status
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");		
		} else {
			setTimeout(function() {loadLPName();}, 100);
			// Create the main viewer.
			var myImg = document.querySelector("#racnav");// mapimg
			var currWidth = myImg.clientWidth;
			var currHeight = myImg.clientHeight;
			alert("Current width=" + currWidth + ", " + "Original height=" + currHeight);
			var viewer = new ROS2D.Viewer({
				divID : 'racnav',
  			width : currWidth,
  			height : currHeight
			});
			// Setup the nav client.
			var nav = NAV2D.OccupancyGridClientNav({
				ros : ros,
  			rootObject : viewer.scene,
  			viewer : viewer,
  			serverName : '/move_base'
			});
			//setTimeout(function() {loadLPName();}, 100);
		}
  });
	oldURL = document.referrer;
	locPause = false;
}

function settingstart() {
	document.getElementById("linearRange").oninput();
	document.getElementById("angularRange").oninput();
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
	oldURL = document.referrer;
	//alert(oldURL);	setTimeout(function() {loadmap();}, 100);
}

function pauseInit() {
	if (!locPause) {
		document.getElementById("pause").innerHTML = "Initialisation of Position Paused. Click <i class='material-icons'>location_searching</i> to Continue.." ;	
		document.getElementById("control").innerHTML = "<i class='material-icons-75'>location_searching</i>";	
		locPause = true;
	} else {
		document.getElementById("pause").innerHTML = "" ;	
		document.getElementById("control").innerHTML = "<i class='material-icons-75'>pan_tool</i>";	
		locPause = false;
	}
}

function goBack() {
	location.replace(oldURL);	
}

function autostart() {
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
	setTimeout(function() {loadmap();}, 100);
	//document.body.addEventListener('click', function(e){
	//	var touchlist = e.touches;
	//}, false);
}
function selectPlan(clicked_id) {
	var tt = clicked_id;
	//alert("---Test---" + tt);
	cmdreq.cmd = 87;	
	cmdreq.gps = tt;
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == 0) {
			location.replace("startpoint.html");
			//location.replace("racNav.html");
		} else {
			if (result.status == 1) {
				location.replace("globotixClean.html");
			} else {
				if (result.status == 2) {
					location.replace("ssmcNav.html");
				} else {
					location.replace("mainmenu.html");
				}
			}
		}
  });
	//setTimeout(function() {location.replace("mainmenu.html");}, 100);
}

function loadLPName() {
	cmdreq.cmd = 88;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == 88) {
			var divElement = document.querySelector('.navmenu');
			numLP = result.numLP;	
			if (numLP > 0) {
				var pElem = document.createElement('p');
				pElem.className = 'navmenu';
				pElem.style.top = "10px";
				pElem.style.height = "50px";
				pElem.style.textAlign = "center";
				pElem.style.fontSize = "150%";
				pElem.innerHTML = "Location";
				divElement.appendChild(pElem);
				for (i=0;i<7;i++) {					
  				pElem = document.createElement('p');
					pElem.className = 'navmenu';
					var bElem = document.createElement('BUTTON');					
					bElem.style.height = "65px";
					bElem.style.width = "196px";
					bElem.style.textAlign = "center";
					bElem.style.fontSize = "250%";
					bElem.setAttribute("id",result.lpname[i]); 
					bElem.onclick = "goLocatiion()";
					bElem.innerHTML = result.lpname[i];
  				pElem.appendChild(bElem);
					divElement.appendChild(pElem);								
				}			
				pElem = document.createElement('p');
				pElem.className = 'navmenu';
				var bElem = document.createElement('BUTTON');					
				bElem.style.height = "65px";
				bElem.style.width = "196px";
				bElem.style.textAlign = "center";
				bElem.style.fontSize = "250%";
				bElem.onclick = "returntoauto()";
				bElem.innerHTML = "<i style='font-size: 55px;' class='material-icons-75'>cancel</i>";
				pElem.appendChild(bElem);
				divElement.appendChild(pElem);	
			}
		} 
  });
}

function loadmap() {
	cmdreq.cmd = 84;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == 84) {
			var tableRef = document.getElementById('plan_table').getElementsByTagName('tbody')[0];	
			numplan = result.numcplan;	
			var trow = tableRef.rows.length;
			var exist = false;
			if (numplan > 0) {
				for (i=0;i<numplan;i++) {					
  				var ix = i;
  				var newRow   = tableRef.insertRow(tableRef.rows.length);
  				// Insert a cell in the row at index 0
  				
					var newCell2  = newRow.insertCell(0);
					newCell2.style.border = "0";
					newCell2.style.textAlign = "left";
					//newCell2.style.margin = "auto";
					//newCell2.style.marginRight = "15px";
					var bu = document.createElement("BUTTON");
					bu.setAttribute("id",result.mapname[i]); 
					bu.style.width ="90px"
					bu.onclick = "selectPlan(this.id)";
					bu.onclick = function(e) {selectPlan(this.id);};
					//bu.addEventListener("onClick", () => { selectPlan(this.id)});
					bu.innerHTML = "<i style='font-size: 50px;' class='material-icons'>touch_app</i>";					
  				newCell2.appendChild(bu);		

					var newCell1  = newRow.insertCell(1);
  				newCell1.style.width = "65%";// 65
					newCell1.style.border =  "0";
					newCell1.style.textAlign =  "center";
					var span = document.createElement("span");
					span.style.fontSize ="150%"
					span.innerHTML = result.cleanplan[i];
  				newCell1.appendChild(span);

					var newCell  = newRow.insertCell(2);
  				newCell.style.width = "180px";
					newCell.style.border =  "0";
					newCell.style.textAlign = "right";
					var img = document.createElement("img");
					img.style.width="180px";
					img.style.height="60px";
					img.src = "cleanplan/"+result.cleanfilename[i];
  				newCell.appendChild(img);
  				
								
				}				
			}
		} 
  });
}

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
			//setTimeout(chkmodeTimer,100);
			location.replace("mainmenu.html");
		} else {
			document.getElementById("status").innerHTML = "Status : Wrong Password ! Try Again" ;	
		}
  });
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

function nav() {
	cmdreq.cmd = 55;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);var kb = document.querySelectorAll(".use-keyboard-input");
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
