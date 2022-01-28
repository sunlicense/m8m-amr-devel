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
var correctFont;
var textHeight;
var textWidth;
var hcorrectFont;
var htextHeight;
var htextWidth;

function startnav() {	
	cmdreq.cmd = 91;	//
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");		
		} else {
			setTimeout(function() {loadLPName();}, 100);			
		}
  });
	oldURL = document.referrer;
	locPause = false;
}

function getTexDim(text) { 
	var sp = document.createElement("span"); 
  document.body.appendChild(sp); 
  sp.style.fontSize = correctFont+"%"; 
  sp.style.height = 'auto'; 
 	sp.style.width = 'auto'; 
  sp.style.position = 'absolute'; 
  sp.style.whiteSpace = 'no-wrap'; 
  sp.innerHTML = text; 
	textWidth = sp.clientWidth;
	textHeight = sp.clientHeight;
  document.body.removeChild(sp); 
}

function findsmallestFont(text,wd,ht) { 
	var sp = document.createElement("span"); 
	var font = 100;
	correctFont = 400;
  document.body.appendChild(sp); 
  //sp.style.fontSize = font+"%"; 
  sp.style.height = 'auto'; 
 	sp.style.width = 'auto'; 
  sp.style.position = 'absolute'; 
  sp.style.whiteSpace = 'no-wrap'; 
	//var len = text.length;
	var i,incr;
	text[8] = 'Location';
	for (i=0;i<8;i++) {
		sp.innerHTML = text[i];
		incr = 2;
		font = 100;
		while(true) {
			sp.style.fontSize = font+"%"; 
			var tW = sp.clientWidth;
			var tH = sp.clientHeight;
			if ((Math.abs(tW-wd) > 10) && (Math.abs(tH-ht) > 10)) {
				incr = 7;
			} else {
				incr = 3;
			}
			if (((tW > wd-10) && (tW < wd)) || ((tH < ht) && (tH > ht-5)) ){
				//correctFont = font;
				break;
			} else {
				if ((tW > wd) || (tH > ht)) {
					font = font - incr;
				} else {
					font = font + incr;
				}
			}
		}
		if (font < correctFont) {
			correctFont = font;
		}
	}
  document.body.removeChild(sp); 
}  

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
	hcorrectFont = font;
	htextHeight = sp.clientHeight;
	htextWidth = sp.clientWidth;
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
			findsmallestFont(result.lpname,140,60);
			numLP = result.numLP;	
			if (numLP > 0) {
				var pElem = document.createElement('p');
				pElem.className = 'navmenu';
				getTextWidth('Location',145,64);
				//getTexDim('Location');
				pElem.style.fontSize = hcorrectFont+"%";// 150				
				var tbm = Math.ceil((63 - htextHeight)/2.0);
				var lrm = Math.ceil((146 - htextWidth)/2.0);
				pElem.style.top = tbm+"px";
				pElem.style.left = lrm+"px";
				pElem.style.right = lrm+"px";
				pElem.style.bottom = tbm+"px";
				pElem.style.height = "63px";
				pElem.style.width = "146px";
				pElem.innerHTML = "Location";
				divElement.appendChild(pElem);			
				//alert(" Width = "+pElem.clientWidth+" Height = "+pElem.clientHeight);
				for (i=0;i<8;i++) {					
  				pElem = document.createElement('p');
					pElem.className = 'navmenu';
					var bElem = document.createElement('BUTTON');			
					//getTextWidth(result.lpname[i],140,60);	
					getTexDim(result.lpname[i]);
					bElem.style.fontSize = correctFont+"%";// 150				
					var tbm = Math.ceil((63 - textHeight)/2.0);
					var lrm = Math.ceil((146 - textWidth)/2.0)
					bElem.style.top = tbm+"px";
					bElem.style.bottom = tbm+"px";
					bElem.style.height = "63px";
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
				bElem.style.height = "63px";
				bElem.style.width = "146px";// 196
				bElem.style.textAlign = "center";
				bElem.style.fontSize = "200%";// 250
				bElem.onclick = function(e) {returntoauto();};
				bElem.innerHTML = "<i style='font-size: 55px;' class='material-icons-75'>home</i>";
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
			var nav = NAV2D.OccupancyGridClientNavNYS({
				ros : ros,
  			rootObject : viewer.scene,
  			viewer : viewer,
  			serverName : '/move_base'
			});
			
		} 
  });
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

function returntoauto() {
	location.replace("mainmenu.html");	
}

function goBack() {
	location.replace(oldURL);	
}

function storeIPInfo() {
	user = document.getElementById("user").value;
	ipaddr = document.getElementById("ipaddr").value;
	passwd = document.getElementById("passwd").value;
	if(typeof(Storage) !== "undefined") {
		localStorage.setItem("user", user);
		localStorage.setItem("ipaddr", ipaddr);
		localStorage.setItem("passwd", passwd);
		location.replace("syncInt.html");
	}	
}

function syncIntstart() {
	cmdreq.cmd = 91;	// 
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");		
		} 
  });
	setTimeout(function() {loaddocmap();}, 100);
	oldURL = document.referrer;
}







