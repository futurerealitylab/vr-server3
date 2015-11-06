chrome.system.display.getInfo(function(dInfos) {
  var width = dInfos[0].workArea.width;
  var height = dInfos[0].workArea.height;
  chrome.app.runtime.onLaunched.addListener(function() {
    chrome.app.window.create('main.html', {
      id: "mainwin",
      innerBounds: {
        width: width,
        height: height
      }
    });
  });
});
