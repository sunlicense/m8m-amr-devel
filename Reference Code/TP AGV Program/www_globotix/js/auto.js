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


function autostart() {
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
	setTimeout(function() {loadmap();}, 100);
}

function selectPlan(clicked_id) {
	var tt = clicked_id;
	//alert("---Test---" + tt);
	cmdreq.cmd = 87;	
	//cmdreq.gps = tt;
	cmdreq.LP = parseInt(tt, 10);
	cmdsrv.callService(cmdreq, function(result) {
    console.log('Result for service call on '
      + cmdsrv.name
      + ': '
      + result.status);
		if (result.status == 87) {
			location.replace("startpoint.html");
			//location.replace("racNav.html");
		} else {
			location.replace("index.html");
		}
  });
	//setTimeout(function() {location.replace("mainmenu.html");}, 100);
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
					//bu.setAttribute("id",result.mapname[i]+"_"+result.cleanplan[i]); 
					bu.setAttribute("id",i);
					bu.style.width ="90px"
					//bu.onclick = "selectPlan(this.id)";
					bu.onclick = function(e) {selectPlan(this.id);};
					//bu.addEventListener("onClick", () => { selectPlan(this.id)});
					bu.innerHTML = "<i style='font-size: 60px;' class='material-icons'>touch_app</i>";					
  				newCell2.appendChild(bu);		

					var newCell1  = newRow.insertCell(1);
  				newCell1.style.width = "60%";// 65
					newCell1.style.border =  "0";
					newCell1.style.textAlign =  "center";
					var span = document.createElement("span");
					span.style.fontSize ="150%"
					span.innerHTML = result.cleanplan[i];
  				newCell1.appendChild(span);

					var newCell  = newRow.insertCell(2);
  				newCell.style.width = "210px";
					newCell.style.border =  "0";
					newCell.style.textAlign = "left";
					//tableRef.offsetWidth = "180px";
					var img = document.createElement("img");
					img.style.width="200px";
					img.style.height="60px";
					img.src = "cleanplan/"+result.cleanfilename[i];
  				newCell.appendChild(img);
  				
								
				}				
			}
		} 
  });
}




