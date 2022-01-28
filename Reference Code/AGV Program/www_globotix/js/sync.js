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

var nummap;
var oldURL;
var user;
var passwd;
var ipaddr;

function syncIPstart() {
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
	oldURL = document.referrer;
	if(typeof(Storage) !== "undefined") {
    user = localStorage.getItem("user");
		document.getElementById("user").value = user;
		ipaddr = localStorage.getItem("ipaddr");
		document.getElementById("ipaddr").value = ipaddr;
		passwd = localStorage.getItem("passwd");		
		document.getElementById("passwd").value = passwd;
  } 
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

function selectMap(clicked_id) {
	var tt = clicked_id;
	document.getElementById("status").innerHTML = "Upload of "+tt+" Started.. Please Wait" ;
	if(typeof(Storage) !== "undefined") {
    user = localStorage.getItem("user");
		ipaddr = localStorage.getItem("ipaddr");
		passwd = localStorage.getItem("passwd");		
  } 
	if ((user != "") && (ipaddr != "") && (passwd != "")) {
		cmdreq.cmd = 95;	
		cmdreq.gps = tt;
		cmdreq.lps = user;
		cmdreq.pw = passwd;
		cmdreq.ip = ipaddr;
		cmdsrv.callService(cmdreq, function(result) {
    	console.log('Result for service call on '
      	+ cmdsrv.name
      	+ ': '
      	+ result.status);
			if (result.status == 95) {
				document.getElementById("status").innerHTML = "Upload of Map Completed" ;
			} else {
				if (result.status == -1) {
					location.replace("index.html");
				}
			}
  	});
	} else {
		document.getElementById("status").innerHTML = "Remote PC Information In-Complete.." ;
	}
}

function deleteMap(clickedid) {
	document.getElementById("status").innerHTML = "Deleting Map "+clickedid+" Started.. Please Wait" ;
	cmdreq.cmd = 96;	// 
	cmdreq.gps = clickedid;
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == -1) {
			location.replace("index.html");		
		} else {
			if (result.status == 96) {
				document.getElementById("status").innerHTML = "Deleting Map Completed.." ;
				setTimeout(function() {loaddocmap();}, 500);
			}
		}
  });
}

function loaddocmap() {
	cmdreq.cmd = 94;	
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == 94) {
			var tableRef = document.getElementById('plan_table').getElementsByTagName('tbody')[0];	
			nummap = result.nummap;				
			var trow = tableRef.rows.length;
			console.log("---- num of map "+nummap+" table rows : "+trow+"  ------"); //deleteRow(0);
			var i;
			for (i = trow - 1; i >= 0; i--) {
        console.log("--- delete row "+i+" trow = "+trow);
				tableRef.deleteRow(i);
      }
			trow = tableRef.rows.length;
			console.log("---- num of map "+nummap+" table rows : "+trow+"  ------"); //deleteRow(0);
			if (nummap > 0) {
				for (i=0;i<nummap;i++) {					
  				var ix = i;
  				var newRow   = tableRef.insertRow(trow);
					var newCell2  = newRow.insertCell(0);
					newCell2.style.border = "0";
					newCell2.style.textAlign = "left";
					var bu = document.createElement("BUTTON");
					bu.setAttribute("id",result.mapname[i]); 
					bu.style.width ="100px"
					bu.onclick = function(e) {selectMap(this.id);};
					bu.innerHTML = "<i style='font-size: 60px;' class='material-icons'>tap_and_play</i>";					
  				newCell2.appendChild(bu);		

					var newCell  = newRow.insertCell(1);
  				newCell.style.width = "100px";
					newCell.style.border =  "0";
					newCell.style.textAlign = "right";
					var bu = document.createElement("BUTTON");
					bu.setAttribute("id",result.mapname[i]); 
					bu.style.width ="90px"
					bu.onclick = function(e) {deleteMap(this.id);};
					bu.innerHTML = "<i style='font-size: 60px;' class='material-icons'>delete</i>";					
  				newCell.appendChild(bu);		  

					var newCell1  = newRow.insertCell(2);
  				newCell1.style.width = "75%";// 65
					newCell1.style.border =  "0";
					newCell1.style.textAlign =  "center";
					var span = document.createElement("span");
					span.style.fontSize ="150%"
					span.innerHTML = result.mapname[i];
  				newCell1.appendChild(span);

													
				}				
			}
		} 
  });
}



