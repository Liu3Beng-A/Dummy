'use strict';
import {Component, h, html, render, useEffect, useState, useRef} from './preact.min.js';

const Message = m => html`<span
  style="color: ${m.message.uart ? '#444' : '#373'};">
  ${m.message.data}
</span>`;

const App = function(props) {
  const [cfg, setCfg] = useState({tcp: {}, ws: {}, mqtt: {}});
  const [messages, setMessages] = useState([]);
  const [connected, setConnected] = useState(false);
  const [txt, setTxt] = useState('');
  const [j1txt, setj1Txt] = useState('0');
  const [j2txt, setj2Txt] = useState('-71');
  const [j3txt, setj3Txt] = useState('180');
  const [j4txt, setj4Txt] = useState('0');
  const [j5txt, setj5Txt] = useState('0');
  const [j6txt, setj6Txt] = useState('0');
  const [velocity, setVelocity] = useState('10');
  const [ws, setWs] = useState(null);
  const refresh = () =>
      fetch('/api/config/get').then(r => r.json()).then(r => setCfg(r));

  const getport = (url, v) => ((url || '').match(/.*:(\d+)/) || ['', v])[1];

  const watchWebsocket = function() {
    // Connect to websocker port, to implement WS console
    var reconnect = function() {
      var port = getport(cfg.ws.url, 4002);
      var l = window.location, proto = l.protocol.replace('http', 'ws');
      var tid, url = `${proto}//${l.hostname}:${port}/ws`;
      // console.log(url);
      var ws = new WebSocket(url);
      ws.onopen = () => {
        setConnected(true);
        setWs(ws);
      };
      ws.onmessage = ev => {
        // console.log(ev, ev.data);
        setMessages(x => x.concat([{data: ev.data, uart: true}]));
      };
      ws.onclose = function() {
        clearTimeout(tid);
        tid = setTimeout(reconnect, 1000);
        setConnected(false);
        setWs(null);
      };
    };
    reconnect();
  };

  useEffect(() => {
    refresh();
    watchWebsocket();
  }, []);


  const sendmessage = ev => {
    setMessages(x => x.concat([{data: txt + '\n', uart: false}]));
    if (ws) ws.send(txt + '\n');
    console.log(txt);
    setTxt('');
  };

  const resetAllJoints = ev => {
    var pos;
      console.log("reset");
      setj1Txt('0');
      document.getElementById('j1show').innerHTML='['+'0'+'] ';

      setj2Txt('-71');
      document.getElementById('j2show').innerHTML='['+'-71'+'] ';

      setj3Txt('180');
      document.getElementById('j3show').innerHTML='['+'180'+'] ';

      setj4Txt('0');
      document.getElementById('j4show').innerHTML='['+'0'+'] ';

      setj5Txt('0');
      document.getElementById('j5show').innerHTML='['+'0'+'] ';

      setj6Txt('0');
      document.getElementById('j6show').innerHTML='['+'0'+'] ';

      setVelocity('10');

      pos = '&'+'0' + ',' + '-71' + ',' + '180' + ',' + '0' + ',' + '0'  + ',' + '0' + ',' + '10'
      setMessages(x => x.concat([{data: pos + '\n', uart: false}]));
      if (ws) ws.send(pos + '\n');
      console.log(pos);
    }

  const sendmessageJoint = ev => {
    var pos;
    var input = ev.target;
    if(input.id === 'j1') {
      console.log("j1");
      setj1Txt(j1txt);
      document.getElementById('j1show').innerHTML='['+j1txt+'] ';
    }else if (input.id === 'j2') {
      console.log("j2");
      setj2Txt(j2txt);
      document.getElementById('j2show').innerHTML='['+j2txt+'] ';
    }else if (input.id === 'j3') {
      console.log("j3");
      setj3Txt(j3txt);
      document.getElementById('j3show').innerHTML='['+j3txt+'] ';

    }else if (input.id === 'j4') {
      console.log("j4");
      setj4Txt(j4txt);
      document.getElementById('j4show').innerHTML='['+j4txt+'] ';

    }else if (input.id === 'j5') {
      console.log("j5");
      setj5Txt(j5txt);
      document.getElementById('j5show').innerHTML='['+j5txt+'] ';

    }else if (input.id === 'j6') {
      console.log("j6");
      setj6Txt(j6txt);
      document.getElementById('j6show').innerHTML='['+j6txt+'] ';
    }else if (input.id === 'velocity') {
      console.log("velocity");
      setVelocity(velocity);
    }
    pos = '&'+j1txt + ',' + j2txt + ',' + j3txt + ',' + j4txt + ',' + j5txt  + ',' + j6txt + ',' + velocity
    setMessages(x => x.concat([{data: pos + '\n', uart: false}]));
    if (ws) ws.send(pos + '\n');
    console.log(pos);
  };

  const onchange = ev => fetch('/api/config/set', {
                           method: 'POST',
                           headers: {'Content-Type': 'application/json'},
                           body: JSON.stringify(cfg),
                         }).then(r => ws && ws.close());

  const set = obj => setCfg(x => Object.assign(x, obj));
  const nset = (n,obj) => setCfg(x => Object.assign(x, {[n]: Object.assign(x[n],obj)}));
  const setTx = ev => set({tx: parseInt(ev.target.value)});
  const setRx = ev => set({rx: parseInt(ev.target.value)});
  const setBaud = ev => set({baud: parseInt(ev.target.value)});
  const setTcpUrl = ev => nset('tcp', {url: `tcp://0.0.0.0:${ev.target.value}`});
  const setWsUrl = ev => nset('ws',{url: `ws://0.0.0.0:${ev.target.value}`});
  const setMqttUrl = ev => nset('mqtt',{url: ev.target.value});
  const setTcpEna = ev => (nset('tcp',{enable: ev.target.checked}), onchange());
  const setWsEna = ev => (nset('ws',{enable: ev.target.checked}), onchange());
  const setMqttEna = ev =>(nset('mqtt', {enable: ev.target.checked}), onchange());
  const setJ1value = ev => nset('j1',{position: ev.target.value});

  return html`
<div class="container">
  <h1 style="margin-bottom: 0;">Dummy WEB CLI </h1>
  <pre class="d-none">${JSON.stringify(cfg, null, 2)}</pre>
  <div class="row">
    <div class="col col-4">
      <h3>UART configuration</h3>
      <div class="d-flex pr-1 my-1">
        <label class="addon">UART TX pin</label>
        <input disabled="disabled" style="width: 5em;" value=${cfg.tx} onchange=${onchange}
          oninput=${setTx} />
      </div><div class="d-flex pr-1 my-1">
        <label class="addon">UART RX pin</label>
        <input disabled="disabled" style="width: 5em;" value=${cfg.rx} onchange=${onchange}
          oninput=${setRx} />
      </div><div class="d-flex pr-1 my-y">
        <label class="addon">UART Baud</label>
        <input style="width: 5em;" value=${cfg.baud} onchange=${onchange}
          oninput=${setBaud} />
      </div>
    </div>
    <div class="col col-8">
      <h3>Network configuration</h3>
      <div class="d-flex my-1">
        <label class="addon">Local TCP port</label>
        <input style="min-width: 4em; flex: 1 100%;"
          value=${getport(cfg.tcp.url, 4001)} onchange=${onchange}
          oninput=${setTcpUrl} />
        <label class="ml-1 d-flex label"><input type="checkbox"
          checked=${cfg.tcp.enable} onchange=${setTcpEna} /> enable</label>
      </div><div class="d-flex my-1">
        <label class="addon">Local WS port</label>
        <input style="flex: 1 100%;"
          value=${getport(cfg.ws.url, 4002)} onchange=${onchange}
          oninput=${setWsUrl} />
        <label class="ml-1 d-flex label"><input type="checkbox"
          checked=${cfg.ws.enable} onchange=${setWsEna} /> enable</label>
      </div><div class="d-flex my-1">
        <label class="addon">Remote MQTT</label>
        <input style="flex: 1 100%;"
          value=${cfg.mqtt.url} onchange=${onchange}
          oninput=${setMqttUrl} />
        <label class="ml-1 d-flex label"><input type="checkbox"
          checked=${cfg.mqtt.enable} onchange=${setMqttEna} /> enable</label>
      </div>
    </div>
  </div>

  <div style="margin-top: 2em;">
    <b>UART console</b><span style="margin-left: 1em; color: #777;">WebSocket status: </span><span
      style="color: ${connected ? 'teal' : 'red'};">
      \u25cf ${connected ? 'connected' : 'disconnected'} </span>
  </div>
  <div style="margin: 0.5em 0; display: flex">
    <input placeholder="to send data, type and press enter..." style="flex: 1 100%;"
      value=${txt} onchange=${sendmessage}
      oninput=${ev => setTxt(ev.target.value)} />
    <button style="margin-left: 1em;"
      onclick=${ev => setMessages([])}>clear</button>
  </div>
  <pre style="height: 15em; overflow: auto;">
    ${messages.map(message => h(Message, {message}))}
  </pre>
  <div class="slidecontainer">
  J1: <input type="range" min="-170" max="170" value=${j1txt} class="slider" id="j1" onchange=${sendmessageJoint} oninput=${ev => setj1Txt(ev.target.value)}/>
    <span id="j1show">[0]</span>
    J2: <input type="range" min="-71" max="71" value=${j2txt} class="slider" id="j2" onchange=${sendmessageJoint} oninput=${ev => setj2Txt(ev.target.value)}/>
    <span id="j2show">[0]</span>
    <br/>J3: <input type="range" min="28" max="180" value=${j3txt} class="slider" id="j3" onchange=${sendmessageJoint} oninput=${ev => setj3Txt(ev.target.value)}/>
    <span id="j3show">[0]</span>
    J4: <input type="range" min="-180" max="180" value=${j4txt} class="slider" id="j4" onchange=${sendmessageJoint} oninput=${ev => setj4Txt(ev.target.value)}/>
    <span id="j4show">[0]</span>
    <br/>J5: <input type="range" min="-120" max="120" value=${j5txt} class="slider" id="j5" onchange=${sendmessageJoint} oninput=${ev => setj5Txt(ev.target.value)}/>
    <span id="j5show">[0]</span>
    J6: <input type="range" min="-720" max="720" value=${j6txt} class="slider" id="j6" onchange=${sendmessageJoint} oninput=${ev => setj6Txt(ev.target.value)}/>
    <span id="j6show">[0]</span>
    <br/>
    Velocity: 
    <input style="min-width: 4em; flex: 1 100%;" id="velocity"
    value=${velocity} onchange=${sendmessageJoint} oninput=${ev => setVelocity(ev.target.value)}
    />
      <button style="margin-left: 1em;" id="j0"
    onclick=${resetAllJoints}>Reset all joints</button>
  </div>  
  
  <div class="msg">
    Commands quick reference:<br/>
    START - enable all joints<br/>
    !DISABLE - disable all joints<br/>
    !STOP - emergency stop<br/>
    !RESET - reset all joints<br/>
    #GETJPOS - get all joints current positions<br/>
    #CMDMODE - 1 SEQUENTIAL 2 INTERRUPTABLE 3 TRAJECTORY<br/>
    &x,x,x,x,x,x,x - set joints position to x, for example &0,-71,180,0,0,0,160 back to home with velocity 160<br/>
    <br/> By MuziXiaowen
    </div>

    </div>`;
};

window.onload = () => render(h(App), document.body);
