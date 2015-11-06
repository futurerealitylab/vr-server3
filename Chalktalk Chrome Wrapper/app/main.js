var webview = null;
var port = null;

/* connect: We establish a connection to the Node.JS native port process. */
function connect() {
  var hostName = "com.mrl.chrome.chalktalk.messaging";
  port = chrome.runtime.connectNative(hostName);
  port.onMessage.addListener(onNativeMessage);
  port.onDisconnect.addListener(onNativeMessageDisconnect);
  console.log("connected to native host");
}
/* "Native Messages" are passed between the Node.JS process and
   this Chrome Application wrapper code. */
function onNativeMessage(message) {
  // console.log("received UDP message: " + message);
  webview.contentWindow.postMessage(message, "*");
}
function onNativeMessageDisconnect() {
  console.log("Failed to connect: " + chrome.runtime.lastError.message);
  port = null;
  //connect();
}

document.addEventListener('DOMContentLoaded', function() {
  connect();
  webview = document.getElementById("webview");
  /* -- Initialize -- */
  /* We initialize the webview's messaging connection.
     It is sent an initial message of "initialize_webview" so that it 
     can grab a handle to our message source and provide us with messages. */
  webview.addEventListener("contentload", function () {
    try{
      webview.contentWindow.postMessage("initialize_webview", "*");
    } catch(error) {
      console.log("initialize_webview error: " + error.stack);
    }
  });
});

var messageHandler = function(event) {
  /* -- Received message from Chalktalk -- */
  /* We now send this message to the Node.JS native port process */
  port.postMessage(event.data);
  // console.log("sent UDP message: " + event.data);
};
window.addEventListener('message', messageHandler, false);