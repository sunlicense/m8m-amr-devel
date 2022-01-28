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
var checkLocalise;

var PM = new ROSLIB.Param({
    ros : ros,
    name : 'mapflag'
});

var localisationListener = new ROSLIB.Topic({
    ros : ros,
    name : '/speed',
    messageType : 'htbot/stat_speed'
  });

localisationListener.subscribe(function(message) {
  //document.getElementById("linear").innerHTML = message.linear.toFixed(2) ; Localised_Ready
	//document.getElementById("angular").innerHTML = message.angular.toFixed(2) ;
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

function checkLocalisation() {
	PM.name = 'localisation_status';
	PM.get(function(value) {
		if (value == 1) {
			document.getElementById("pause").innerHTML = "Localising Start Position. Please wait...";
			PM.name = 'Localised_Ready';
			PM.get(function(value) {
				if (value) {
					alert("--- checking localisation done----");
					PM.name = 'init_localisation';
					PM.set(false);
					clearInterval(checkLocalise);
					location.replace("racNav.html");
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
			location.replace("racNav.html");
		} else {
			document.getElementById("pause").innerHTML = "Localisation of Start Position Not Done. Please wait...";
		}
	});
}

function startlocalise() {
	cmdreq.cmd = 89;	// check login status
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
			//alert(" Width = "+window.innerWidth);
			/*
			var viewer = new ROS2D.Viewer({
				divID : 'racnav',
  			width : 500,
  			height : 300,
			});
			// Setup the nav client.
			var nav = NAV2D.OccupancyGridClientNav({
				ros : ros,
  			rootObject : viewer.scene,
  			viewer : viewer,
  			serverName : '/move_base'
			});
			*/
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
		PM.name = 'localisation_status';
		PM.set(99);
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

function movetostn(clicked_id) {
	cmdreq.cmd = 11;	
	cmdreq.lps = clicked_id;
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == 11) {
			alert(" Move to Station OK ");
		} else {
			alert(" Move to Station Failed ");
		}
  });
}

var correctFont;
var textHeight;
var textWidth;

function getTextWidth(text,size,height) { 
	var sp = document.createElement("span"); 
	var font = 100;
  document.body.appendChild(sp); 
  //sp.style.fontSize = font+"%"; 
  sp.style.height = 'auto'; 
 	sp.style.width = 'auto'; 
  sp.style.position = 'absolute'; 
  sp.style.whiteSpace = 'no-wrap'; 
  sp.innerHTML = text; 
	while(true) {
		sp.style.fontSize = font+"%"; 
		var textWidth = sp.clientWidth;
		var tH = sp.clientHeight;
		if (((textWidth > size-10) && (textWidth < size)) || ((tH < height) && (tH > height-5)) ){
			//correctFont = font;
			break;
		} else {
			if ((textWidth > size) || (tH > height)) {
				font = font - 2;
			} else {
				font = font + 2;
			}
		}
	}
	correctFont = font;
	textHeight = sp.clientHeight;
	textWidth = sp.clientWidth;
  //textWidth = sp.clientWidth; //Math.ceil(sp.clientWidth); 
	//document.getElementById("status").appendChild(sp) ;	
  document.body.removeChild(sp); 
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
				getTextWidth('Location',140,60);
				pElem.style.fontSize = correctFont+"%";// 150				
				var tbm = Math.ceil((60 - textHeight)/2.0);
				var lrm = Math.ceil((146 - textWidth)/2.0);
				pElem.style.top = tbm+"px";
				pElem.style.left = lrm+"px";
				pElem.style.right = lrm+"px";
				pElem.style.bottom = tbm+"px";
				pElem.style.height = "60px";
				//pElem.style.textAlign = "left";
				pElem.innerHTML = "Location";
				divElement.appendChild(pElem);			
				//alert(" Width = "+pElem.clientWidth+" Height = "+pElem.clientHeight);
				for (i=0;i<7;i++) {					
  				pElem = document.createElement('p');
					pElem.className = 'navmenu';
					var bElem = document.createElement('BUTTON');			
					getTextWidth(result.lpname[i],140,60);		
					bElem.style.fontSize = correctFont+"%";// 150				
					var tbm = Math.ceil((60 - textHeight)/2.0);
					var lrm = Math.ceil((146 - textWidth)/2.0)
					bElem.style.top = tbm+"px";
					bElem.style.bottom = tbm+"px";
					bElem.style.height = "60px";
					bElem.style.width = "146px";// 196
					bElem.style.left = lrm+"px";
					bElem.style.right = lrm+"px";
					//bElem.style.textAlign = "center";
					//bElem.style.fontSize = "200%"; // 250
					bElem.setAttribute("id",result.lpname[i]); 
					bElem.onclick = function(e) {movetostn(this.id);};
					bElem.innerHTML = result.lpname[i];
  				pElem.appendChild(bElem);
					divElement.appendChild(pElem);								
				}			
				pElem = document.createElement('p');
				pElem.className = 'navmenu';
				var bElem = document.createElement('BUTTON');					
				bElem.style.height = "65px";
				bElem.style.width = "146px";// 196
				bElem.style.textAlign = "center";
				bElem.style.fontSize = "200%";// 250
				bElem.onclick = function(e) {returntoauto();};
				bElem.innerHTML = "<i style='font-size: 55px;' class='material-icons-75'>cancel</i>";
				pElem.appendChild(bElem);
				divElement.appendChild(pElem);	
			}
			var myImg = document.querySelector("#racnav");// mapimg
			var currWidth = myImg.clientWidth;
			var currHeight = myImg.clientHeight;
			//alert("Current width=" + currWidth + ", " + "Original height=" + currHeight);			
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
					//bu.onclick = "selectPlan(this.id)";
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
