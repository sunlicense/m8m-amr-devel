var main;
var keysContainer;
var keys = [];
var eventHandlers = {
	oninput: null,
  onclose: null,
};
var oninput;
var onclose;
var value;
var capsLock;
var fragment;
var curKey;

var keyLayout_org = [
	"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "backspace",
  "q", "w", "e", "r", "t", "y", "u", "i", "o", "p",
  "caps", "a", "s", "d", "f", "g", "h", "j", "k", "l", "enter",
  "done", "z", "x", "c", "v", "b", "n", "m", ",", ".", "?",
  "space"
];

var keyLayout = [
	"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "backspace",
  "q", "w", "e", "r", "t", "y", "u", "i", "o", "p",
  "caps", "a", "s", "d", "f", "g", "h", "j", "k", "l",
  "done", "z", "x", "c", "v", "b", "n", "m",  ".",
];

function createIconHTML(icon_name) {
	return "<i class='material-icons'>"+icon_name+"</i>";
}

function  _toggleCapsLock() {
	capsLock = !capsLock;
  keys.forEach(key => {
  	if (key.childElementCount === 0) {
    	key.textContent = capsLock ? key.textContent.toUpperCase() : key.textContent.toLowerCase();
    }
	})
}

function _triggerEvent(handlerName) {
	var ty = typeof handlerName;
	if (typeof eventHandlers[handlerName] == "function") {
  	eventHandlers[handlerName](value);
    console.log("triggered event : "+value);
  }
}

function Keyopen(initialValue, oninp, oncl) {
	value = initialValue || "";
 	eventHandlers.oninput = oninp;
 	eventHandlers.onclose = oncl;
  main.classList.remove("keyboard--hidden");
}

function Keyclose() {    		
	console.log(" closing ");
 	value = "";
	eventHandlers.oninput = oninput;
 	eventHandlers.onclose = onclose;
 	main.classList.add("keyboard--hidden"); 	
}

function KeyclearInput() {
	//console.log(" Clear Input ");
	document.getElementById("pass").value = "";
	document.getElementById("status").innerHTML = " " ;			
	value = "";
	//_triggerEvent("oninput");
}

function KeyclearInputUser() {
	document.getElementById("user").value = "";
	//document.getElementById("status").innerHTML = " " ;			
	value = "";
	//_triggerEvent("oninput");
}

function KeyclearInputIP() {
	document.getElementById("ipaddr").value = "";
	//document.getElementById("status").innerHTML = " " ;			
	value = "";
	//_triggerEvent("oninput");
}  

function KeyclearInputPW() {
	document.getElementById("passwd").value = "";
	//document.getElementById("status").innerHTML = " " ;			
	value = "";
	//_triggerEvent("oninput");
} 
   
function _createKeys() {
	var keyElement;
	var insertLineBreak;
	fragment = document.createDocumentFragment(); 
  keyLayout.forEach(key => {
  	keyElement = document.createElement("button");
    //const insertLineBreak = ["backspace", "p", "enter", "?"].indexOf(key) !== -1;
    insertLineBreak = ["backspace", "p", "l"].indexOf(key) !== -1;
    // Add attributes/classes
    keyElement.setAttribute("type", "button");
    keyElement.classList.add("keyboard__key");

    switch (key) {
    	case "backspace":
      	keyElement.classList.add("keyboard__key--wide");
        keyElement.innerHTML = createIconHTML("backspace");
        keyElement.addEventListener("click", () => {
        	value = value.substring(0,value.length - 1);
          _triggerEvent("oninput");
        });
        break;
   		case "caps":
    		keyElement.classList.add("keyboard__key--wide", "keyboard__key__activatable");
      	keyElement.innerHTML = createIconHTML("keyboard_capslock");
      	curKey = keyElement;
      	keyElement.addEventListener("click", () => {
      		_toggleCapsLock();
        	//keyElement.classList.toggle("keyboard__key--active",capsLock);
        	if (capsLock) {
        		curKey.classList.add("keyboard__key--active");
        	} else {
        		curKey.classList.remove("keyboard__key--active");
        	}
      	});
      	break;
  		case "enter":
   			keyElement.classList.add("keyboard__key--wide");
      	keyElement.innerHTML = createIconHTML("keyboard_return");
      	keyElement.addEventListener("click", () => {
      		value += "\n";
       	 _triggerEvent("oninput");
      	});
      	break;
  		case "space":
    		keyElement.classList.add("keyboard__key--extra-wide");
      	keyElement.innerHTML = createIconHTML("space_bar");
   			keyElement.addEventListener("click", () => {
      		value += " ";
       		_triggerEvent("oninput");
     		});
				break;
			case "done":
    		keyElement.classList.add("keyboard__key--wide", "keyboard__key--dark");
     		keyElement.innerHTML = createIconHTML("keyboard_hide");//exit_to_app
				keyElement.addEventListener("click", () => {
      		Keyclose();                        
       		_triggerEvent("onclose");
     		});
     		break;
   		default:
     		keyElement.textContent = key.toLowerCase();
     		keyElement.addEventListener("click", () => {
      		value += capsLock ? key.toUpperCase() : key.toLowerCase();                       
       		_triggerEvent("oninput");                       
     		});
				break;
 		}
		fragment.appendChild(keyElement);
		if (insertLineBreak) {
    	fragment.appendChild(document.createElement("br"));
    }
 	});
  return fragment;
}

function Keyinit() {
	// Create main elements
  main = document.createElement("div");
  keysContainer = document.createElement("div");
  // Setup main elements
  main.classList.add("keyboard", "keyboard--hidden");
  keysContainer.classList.add("keyboard__keys");
  keysContainer.appendChild(_createKeys());
  keys = keysContainer.querySelectorAll(".keyboard__key");
  // Add to DOM
  main.appendChild(keysContainer);
  document.body.appendChild(main);
  // Automatically use keyboard for elements with .use-keyboard-input
  var kb = document.querySelectorAll(".use-keyboard-input");
  capsLock = false;
  kb.forEach(element => {
  	element.addEventListener("focus", () => {
    	Keyopen(element.value, currentValue => {
      	element.value = currentValue;
      }, currentValue => {
      	element.value = currentValue;
      });
    });
  });
}

window.addEventListener("DOMContentLoaded", function () {
	Keyinit();
});
