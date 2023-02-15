var term = null;
// just keep a global ref to the instance around for convenience
var instance;
// this function will be imported for wasm to use
function tty_write(c) {
    //console.print(String.fromCharCode(c));
    term.write(String.fromCharCode(c));
}

function getTimeUs() {
    return window.performance.now() * 1000;
}
function sleep() {
    //console.log("SLEEP");
}

// define our imports
var imports = {
    env: {
        tty_write: tty_write,
        getTimeUs: getTimeUs,
        sleep: sleep
    }
};
// do the thing
fetch("wasmtest.wasm").then(function (response) {
    return response.arrayBuffer();
}).then(function (bytes) {
    return WebAssembly.instantiate(bytes, imports);
}).then(function (results) {
    instance = results.instance;
    // grab our exported function from wasm
    var setup = results.instance.exports.setup;
    var loop = results.instance.exports.loop;
    var tty_read = results.instance.exports.tty_read;

    term = new Terminal();
    term.open(document.getElementById('terminal'));

    term.onKey(function (ev) {
        const char = ev; 
        let utf8Encode = new TextEncoder();
        enc = utf8Encode.encode(ev.key);
        tty_read(enc[0]);
    });

    term.write('Booting Linux...\r\n')

    setup();
    setInterval(() => {
        for (var i=0;i<10;i++) {
            loop();
        }
    }, 0);
    console.log("GO done");
}).catch((err) => {
    console.log(err);
});
