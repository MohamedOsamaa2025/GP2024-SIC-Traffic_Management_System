[
    {
        "id": "8b886473a825886d",
        "type": "tab",
        "label": "Flow 1",
        "disabled": false,
        "info": "",
        "env": []
    },
    {
        "id": "15c696571ddb5569",
        "type": "mqtt out",
        "z": "8b886473a825886d",
        "name": "",
        "topic": "AdaptiveLights1",
        "qos": "1",
        "retain": "",
        "respTopic": "",
        "contentType": "",
        "userProps": "",
        "correl": "",
        "expiry": "",
        "broker": "856744da1a0ab30d",
        "x": 2253.214385986328,
        "y": 1105.714376449585,
        "wires": []
    },
    {
        "id": "67f7cb620e2a3c76",
        "type": "change",
        "z": "8b886473a825886d",
        "name": "",
        "rules": [
            {
                "t": "set",
                "p": "topic",
                "pt": "msg",
                "to": "pushbullet/alert",
                "tot": "str"
            }
        ],
        "action": "",
        "property": "",
        "from": "",
        "to": "",
        "reg": false,
        "x": 850,
        "y": 1200,
        "wires": [
            [
                "7609fdbef2af1fb6"
            ]
        ]
    },
    {
        "id": "2e62729a500452a9",
        "type": "mqtt out",
        "z": "8b886473a825886d",
        "name": "",
        "topic": "em",
        "qos": "1",
        "retain": "",
        "respTopic": "",
        "contentType": "",
        "userProps": "",
        "correl": "",
        "expiry": "",
        "broker": "856744da1a0ab30d",
        "x": 830.7143859863281,
        "y": 1255.714376449585,
        "wires": []
    },
    {
        "id": "648e17a7b35dbc08",
        "type": "mqtt in",
        "z": "8b886473a825886d",
        "name": "road_1",
        "topic": "road_1",
        "qos": "1",
        "datatype": "auto-detect",
        "broker": "856744da1a0ab30d",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 460.7143859863281,
        "y": 1495.714376449585,
        "wires": [
            [
                "36e32e7ddb763cc1"
            ]
        ]
    },
    {
        "id": "769f21e159c0cc22",
        "type": "mqtt in",
        "z": "8b886473a825886d",
        "name": "road_2",
        "topic": "road_2",
        "qos": "1",
        "datatype": "auto-detect",
        "broker": "856744da1a0ab30d",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 460.7143859863281,
        "y": 1575.714376449585,
        "wires": [
            [
                "d3110e9e104fd763"
            ]
        ]
    },
    {
        "id": "13abc9af6aaff974",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 18",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 840,
        "y": 1080,
        "wires": []
    },
    {
        "id": "36e32e7ddb763cc1",
        "type": "function",
        "z": "8b886473a825886d",
        "name": "function 2",
        "func": "flow.set('value1', parseFloat(msg.payload));  // Store value1 in flow context\nreturn msg;\n",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 670.7143859863281,
        "y": 1495.714376449585,
        "wires": [
            [
                "1b6f250b47985225",
                "74bc0e49f663df84"
            ]
        ]
    },
    {
        "id": "d3110e9e104fd763",
        "type": "function",
        "z": "8b886473a825886d",
        "name": "function 3",
        "func": "flow.set('value2', parseFloat(msg.payload));  // Store value2 in flow context\nreturn msg;\n",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 670.7143859863281,
        "y": 1575.714376449585,
        "wires": [
            [
                "1b6f250b47985225",
                "dfeb1c7d9961bdfc"
            ]
        ]
    },
    {
        "id": "1b6f250b47985225",
        "type": "function",
        "z": "8b886473a825886d",
        "name": "function 4",
        "func": "let value1 = flow.get('value1'); \nlet value2 = flow.get('value2');\n\n// Check if both values are available\nif (value1 !== null && value2 !== null) {\n    if (value1 < value2) {\n        msg.payload = `road1`;  // Send 'road1' if value1 is greater\n    } else if (value2 < value1) {\n        msg.payload = `road2`;  // Send 'road2' if value2 is greater\n    }\n\n// Clear the stored values\nflow.set('value1', null);\nflow.set('value2', null);\n\nreturn msg;\n\n} else {\n    // Store the incoming value if available\n    if (msg.payload.value1 !== undefined) {\n        flow.set('value1', msg.payload.value1);\n    }\n    if (msg.payload.value2 !== undefined) {\n        flow.set('value2', msg.payload.value2);\n    }\n\n    // Return null to indicate waiting for both values\n    return null;\n}\n",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 870.7143859863281,
        "y": 1535.714376449585,
        "wires": [
            [
                "8c1e868b4dd9eeeb",
                "5d729e508f6b9bb3",
                "614a62d98c5b978a"
            ]
        ]
    },
    {
        "id": "614a62d98c5b978a",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 19",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 1100.7143859863281,
        "y": 1615.714376449585,
        "wires": []
    },
    {
        "id": "f839d4119a4aa017",
        "type": "pushbullet in",
        "z": "8b886473a825886d",
        "config": "2a1b726d1c898952",
        "name": "Pushbullet Emergency",
        "x": 520.7143859863281,
        "y": 1195.714376449585,
        "wires": [
            [
                "2e62729a500452a9",
                "13abc9af6aaff974",
                "67f7cb620e2a3c76",
                "0ce95131b0f446cd"
            ]
        ]
    },
    {
        "id": "7609fdbef2af1fb6",
        "type": "function",
        "z": "8b886473a825886d",
        "name": "Traffic & Emergency Control",
        "func": "var msg1 = {};  // Road 1 traffic light message\nvar msg2 = {};  // Road 2 traffic light message\nvar system_mode = global.get(\"system_mode\") || \"normal\";  // Track mode (normal or emergency)\nvar accident_mode = global.get(\"accident_mode\") || false;  // Accident detection mode\n\n// Handle MQTT traffic control when in normal mode\nif (system_mode === \"normal\" && !accident_mode && msg.topic === \"TrafficDensity\") {\n    switch (msg.payload) {\n        case \"road1\":\n            global.set(\"color_road1\", \"green\");\n            global.set(\"color_road2\", \"red\");\n            break;\n        case \"road2\":\n            global.set(\"color_road1\", \"red\");\n            global.set(\"color_road2\", \"green\");\n            break;\n        case \"yellow1\":\n            global.set(\"color_road1\", \"yellow\");\n            global.set(\"color_road2\", \"green\");\n            break;\n        case \"yellow2\":\n            global.set(\"color_road2\", \"yellow\");\n            global.set(\"color_road1\", \"green\");\n            break;\n    }\n    // Save last status in case of an emergency interrupt or accident\n    global.set(\"last_traffic_status\", msg.payload);\n}\n\n// Handle Pushbullet emergency interrupts\nif (msg.topic === \"pushbullet/alert\") {\n    if (msg.payload === \"r1\") {\n        global.set(\"system_mode\", \"emergency\");\n        global.set(\"color_road1\", \"green\");\n        global.set(\"color_road2\", \"red\");\n    } else if (msg.payload === \"r2\") {\n        global.set(\"system_mode\", \"emergency\");\n        global.set(\"color_road1\", \"red\");\n        global.set(\"color_road2\", \"green\");\n    } else if (msg.payload === \"done\") {\n        global.set(\"system_mode\", \"normal\");\n        var last_status = global.get(\"last_traffic_status\") || \"road1\";\n        switch (last_status) {\n            case \"road1\":\n                global.set(\"color_road1\", \"green\");\n                global.set(\"color_road2\", \"red\");\n                break;\n            case \"road2\":\n                global.set(\"color_road1\", \"red\");\n                global.set(\"color_road2\", \"green\");\n                break;\n            case \"yellow1\":\n                global.set(\"color_road1\", \"yellow\");\n                global.set(\"color_road2\", \"green\");\n                break;\n            case \"yellow2\":\n                global.set(\"color_road2\", \"yellow\");\n                global.set(\"color_road1\", \"green\");\n                break;\n        }\n    }\n}\n\n// Accident detection mode\nif (msg.topic === \"accident/detect\") {\n    global.set(\"accident_mode\", true);\n    if (msg.payload === \"road1\") {\n        global.set(\"color_road1\", \"red\");\n        global.set(\"color_road2\", \"green\");  // Set road1 light to red in case of an accident\n    } else if (msg.payload === \"road2\") {\n        global.set(\"color_road1\", \"green\");\n        global.set(\"color_road2\", \"red\");  // Set road2 light to red in case of an accident\n    }\n}\n\n// Accident recovery mode (reset accident_mode to false)\nif (msg.topic === \"accident/recover\") {\n    global.set(\"accident_mode\", false);\n    \n    // Restore last traffic status before the accident\n    var last_status = global.get(\"last_traffic_status\") || \"road1\";\n    switch (last_status) {\n        case \"road1\":\n            global.set(\"color_road1\", \"green\");\n            global.set(\"color_road2\", \"red\");\n            break;\n        case \"road2\":\n            global.set(\"color_road1\", \"red\");\n            global.set(\"color_road2\", \"green\");\n            break;\n        case \"yellow1\":\n            global.set(\"color_road1\", \"yellow\");\n            global.set(\"color_road2\", \"green\");\n            break;\n        case \"yellow2\":\n            global.set(\"color_road2\", \"yellow\");\n            global.set(\"color_road1\", \"green\");\n            break;\n    }\n}\n\nmsg1.topic = \"AdaptiveLights1\";\nmsg1.payload = global.get(\"color_road1\");\n\nmsg2.topic = \"AdaptiveLights2\";\nmsg2.payload = global.get(\"color_road2\");\n\nreturn [msg1, msg2];\n",
        "outputs": 2,
        "timeout": "",
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 1813.2144470214844,
        "y": 1198.2144298553467,
        "wires": [
            [
                "15c696571ddb5569",
                "052029fe9ed53c0f",
                "cb017fb87cfc7fd3"
            ],
            [
                "7abb0b22b8b08623",
                "03817befca5f764f",
                "e0efdc0aa87824a1"
            ]
        ]
    },
    {
        "id": "8c1e868b4dd9eeeb",
        "type": "change",
        "z": "8b886473a825886d",
        "name": "",
        "rules": [
            {
                "t": "set",
                "p": "topic",
                "pt": "msg",
                "to": "accident/detect",
                "tot": "str"
            }
        ],
        "action": "",
        "property": "",
        "from": "",
        "to": "",
        "reg": false,
        "x": 1120.7143859863281,
        "y": 1415.714376449585,
        "wires": [
            [
                "7609fdbef2af1fb6",
                "e112d3dd3cd020e1"
            ]
        ]
    },
    {
        "id": "5d729e508f6b9bb3",
        "type": "mqtt out",
        "z": "8b886473a825886d",
        "name": "",
        "topic": "ACC",
        "qos": "1",
        "retain": "",
        "respTopic": "",
        "contentType": "",
        "userProps": "",
        "correl": "",
        "expiry": "",
        "broker": "856744da1a0ab30d",
        "x": 1390,
        "y": 1540,
        "wires": []
    },
    {
        "id": "285cfd188ef2849c",
        "type": "ui_button",
        "z": "8b886473a825886d",
        "name": "Send Alert",
        "group": "626a51c24821ce56",
        "order": 30,
        "width": 6,
        "height": 1,
        "passthru": false,
        "label": "Retuen To Normal System",
        "tooltip": "",
        "color": "",
        "bgcolor": "",
        "icon": "",
        "payload": "safe",
        "payloadType": "str",
        "topic": "pushbullet/alert",
        "topicType": "str",
        "x": 1410.7143859863281,
        "y": 1415.714376449585,
        "wires": [
            [
                "92b617c1871a7f43",
                "d5c65b51d51222cf",
                "5d729e508f6b9bb3"
            ]
        ]
    },
    {
        "id": "92b617c1871a7f43",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 21",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "false",
        "statusVal": "",
        "statusType": "auto",
        "x": 1640.7143859863281,
        "y": 1475.714376449585,
        "wires": []
    },
    {
        "id": "86256ccb3b23ab7d",
        "type": "mqtt in",
        "z": "8b886473a825886d",
        "name": "TrafficDensity1",
        "topic": "TrafficDensity1",
        "qos": "1",
        "datatype": "auto-detect",
        "broker": "856744da1a0ab30d",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 500.7143859863281,
        "y": 835.714376449585,
        "wires": [
            [
                "0b80c7784e4bb25d",
                "eca317b432495cab"
            ]
        ]
    },
    {
        "id": "fccdde551a4b2fe3",
        "type": "mqtt in",
        "z": "8b886473a825886d",
        "name": "TrafficDensity2",
        "topic": "TrafficDensity2",
        "qos": "1",
        "datatype": "auto-detect",
        "broker": "856744da1a0ab30d",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 500.7143859863281,
        "y": 915.714376449585,
        "wires": [
            [
                "1b4f98a658a1eb0b",
                "eca317b432495cab"
            ]
        ]
    },
    {
        "id": "0b80c7784e4bb25d",
        "type": "function",
        "z": "8b886473a825886d",
        "name": "function 5",
        "func": "flow.set('value1', msg.payload);  // Store value1 as a string\nreturn msg;\n",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 700.7143859863281,
        "y": 835.714376449585,
        "wires": [
            [
                "910078b98f5ac208",
                "a92805038d710d55"
            ]
        ]
    },
    {
        "id": "1b4f98a658a1eb0b",
        "type": "function",
        "z": "8b886473a825886d",
        "name": "function 6",
        "func": "flow.set('value2', msg.payload);  // Store value2 as a string\nreturn msg;\n",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 700.7143859863281,
        "y": 915.714376449585,
        "wires": [
            [
                "b923c244ad87d64d",
                "a92805038d710d55"
            ]
        ]
    },
    {
        "id": "7abb0b22b8b08623",
        "type": "mqtt out",
        "z": "8b886473a825886d",
        "name": "",
        "topic": "AdaptiveLights2",
        "qos": "1",
        "retain": "",
        "respTopic": "",
        "contentType": "",
        "userProps": "",
        "correl": "",
        "expiry": "",
        "broker": "856744da1a0ab30d",
        "x": 2253.214385986328,
        "y": 1245.714376449585,
        "wires": []
    },
    {
        "id": "a5383e28262395f7",
        "type": "change",
        "z": "8b886473a825886d",
        "name": "",
        "rules": [
            {
                "t": "set",
                "p": "topic",
                "pt": "msg",
                "to": "TrafficDensity",
                "tot": "str"
            }
        ],
        "action": "",
        "property": "",
        "from": "",
        "to": "",
        "reg": false,
        "x": 1120.7143859863281,
        "y": 875.714376449585,
        "wires": [
            [
                "7609fdbef2af1fb6"
            ]
        ]
    },
    {
        "id": "e112d3dd3cd020e1",
        "type": "switch",
        "z": "8b886473a825886d",
        "name": "",
        "property": "payload",
        "propertyType": "msg",
        "rules": [
            {
                "t": "eq",
                "v": "road1",
                "vt": "str"
            },
            {
                "t": "eq",
                "v": "road2",
                "vt": "str"
            }
        ],
        "checkall": "true",
        "repair": false,
        "outputs": 2,
        "x": 1390,
        "y": 1640,
        "wires": [
            [
                "71df008e87f64f4d"
            ],
            [
                "64e5f0b01e69b595"
            ]
        ]
    },
    {
        "id": "71df008e87f64f4d",
        "type": "pushbullet",
        "z": "8b886473a825886d",
        "config": "2a1b726d1c898952",
        "pushtype": "",
        "title": "Emergency Alert >> Accident Detected At Road 1",
        "chan": "",
        "name": "Send Push Notification",
        "x": 1620.7143859863281,
        "y": 1595.714376449585,
        "wires": []
    },
    {
        "id": "64e5f0b01e69b595",
        "type": "pushbullet",
        "z": "8b886473a825886d",
        "config": "2a1b726d1c898952",
        "pushtype": "",
        "title": "Emergency Alert >> Accident Detected At Road 2",
        "chan": "",
        "name": "Send Push Notification",
        "x": 1620.7143859863281,
        "y": 1675.714376449585,
        "wires": []
    },
    {
        "id": "052029fe9ed53c0f",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 22",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 2233.214385986328,
        "y": 1045.714376449585,
        "wires": []
    },
    {
        "id": "910078b98f5ac208",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 23",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 900.7143859863281,
        "y": 775.714376449585,
        "wires": []
    },
    {
        "id": "b923c244ad87d64d",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 25",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 900.7143859863281,
        "y": 975.714376449585,
        "wires": []
    },
    {
        "id": "03817befca5f764f",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 27",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 2233.214385986328,
        "y": 1305.714376449585,
        "wires": []
    },
    {
        "id": "74bc0e49f663df84",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 30",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "false",
        "statusVal": "",
        "statusType": "auto",
        "x": 870.7143859863281,
        "y": 1455.714376449585,
        "wires": []
    },
    {
        "id": "dfeb1c7d9961bdfc",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 31",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "false",
        "statusVal": "",
        "statusType": "auto",
        "x": 870.7143859863281,
        "y": 1635.714376449585,
        "wires": []
    },
    {
        "id": "a92805038d710d55",
        "type": "function",
        "z": "8b886473a825886d",
        "name": "function 8",
        "func": "// Retrieve the stored values\nlet value1 = flow.get('value1');\nlet value2 = flow.get('value2');\n\n// Check if both values are available\nif (value1 !== null && value2 !== null) {\n    let density_map = { low: 1, medium: 2, high: 3 };\n\n    let density1 = density_map[value1];  \n    let density2 = density_map[value2];  \n\n    // Determine which road is busier\n    if (value1 === \"medium\") {\n        msg.payload = `yellow1`;  \n    } \n    else if (value2 === \"medium\") {\n        msg.payload = `yellow2`;  \n    }    \n    else if (density1 > density2) {\n        msg.payload = `road1`;\n    } else if (density2 > density1) {\n        msg.payload = `road2`;\n    }\n    // Clear the stored values\n    flow.set('value1', null);\n    flow.set('value2', null);\n\n    return msg;  \n} else {\n    // If either value is missing, store the current value\n    if (msg.payload.value1 !== undefined) {\n        flow.set('value1', msg.payload.value1);\n    }\n    if (msg.payload.value2 !== undefined) {\n        flow.set('value2', msg.payload.value2);\n    }\n\n    // Return null to indicate waiting for both values\n    return null;\n}\n",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 920.7143859863281,
        "y": 875.714376449585,
        "wires": [
            [
                "a5383e28262395f7",
                "e64859bcfca2f3a9"
            ]
        ]
    },
    {
        "id": "e64859bcfca2f3a9",
        "type": "debug",
        "z": "8b886473a825886d",
        "name": "debug 32",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "false",
        "statusVal": "",
        "statusType": "auto",
        "x": 1100.7143859863281,
        "y": 795.714376449585,
        "wires": []
    },
    {
        "id": "d5c65b51d51222cf",
        "type": "change",
        "z": "8b886473a825886d",
        "name": "",
        "rules": [
            {
                "t": "set",
                "p": "topic",
                "pt": "msg",
                "to": "accident/recover",
                "tot": "str"
            }
        ],
        "action": "",
        "property": "",
        "from": "",
        "to": "",
        "reg": false,
        "x": 1640.7143859863281,
        "y": 1415.714376449585,
        "wires": [
            [
                "7609fdbef2af1fb6"
            ]
        ]
    },
    {
        "id": "ba2cbe214ea47b97",
        "type": "inject",
        "z": "8b886473a825886d",
        "name": "",
        "props": [
            {
                "p": "payload"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "12",
        "payloadType": "num",
        "x": 470.7143859863281,
        "y": 1655.714376449585,
        "wires": [
            [
                "d3110e9e104fd763"
            ]
        ]
    },
    {
        "id": "ae8382e294b84872",
        "type": "inject",
        "z": "8b886473a825886d",
        "name": "",
        "props": [
            {
                "p": "payload"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "10",
        "payloadType": "num",
        "x": 470.7143859863281,
        "y": 1415.714376449585,
        "wires": [
            [
                "36e32e7ddb763cc1"
            ]
        ]
    },
    {
        "id": "eca317b432495cab",
        "type": "function",
        "z": "8b886473a825886d",
        "name": "Convert Density",
        "func": "var road1_msg = {};  // Output for Road 1 gauge\nvar road2_msg = {};  // Output for Road 2 gauge\n\nif (msg.topic === \"TrafficDensity1\") {\n    switch (msg.payload) {\n        case \"low\":\n            road1_msg.payload = 33;\n            break;\n        case \"medium\":\n            road1_msg.payload = 66;\n            break;\n        case \"high\":\n            road1_msg.payload = 100;\n            break;\n        default:\n            road1_msg.payload = 0;\n    }\n    road1_msg.topic = \"Road 1 Traffic Density\";\n}\n\nif (msg.topic === \"TrafficDensity2\") {\n    switch (msg.payload) {\n        case \"low\":\n            road2_msg.payload = 33;\n            break;\n        case \"medium\":\n            road2_msg.payload = 66;\n            break;\n        case \"high\":\n            road2_msg.payload = 100;\n            break;\n        default:\n            road2_msg.payload = 0;\n    }\n    road2_msg.topic = \"Road 2 Traffic Density\";\n}\n\nreturn [road1_msg, road2_msg];",
        "outputs": 2,
        "timeout": "",
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 720.7143859863281,
        "y": 655.714376449585,
        "wires": [
            [
                "669e86c0f67bf5ce"
            ],
            [
                "986ff805d8f86b2d"
            ]
        ]
    },
    {
        "id": "669e86c0f67bf5ce",
        "type": "ui_gauge",
        "z": "8b886473a825886d",
        "name": "",
        "group": "626a51c24821ce56",
        "order": 33,
        "width": 6,
        "height": 4,
        "gtype": "gage",
        "title": "Traffic Density Road 1",
        "label": "%",
        "format": "{{value}}%",
        "min": 0,
        "max": 100,
        "colors": [
            "#00b500",
            "#e5e500",
            "#ca3838"
        ],
        "seg1": "",
        "seg2": "",
        "x": 920.7143859863281,
        "y": 625.714376449585,
        "wires": []
    },
    {
        "id": "986ff805d8f86b2d",
        "type": "ui_gauge",
        "z": "8b886473a825886d",
        "name": "",
        "group": "626a51c24821ce56",
        "order": 35,
        "width": 6,
        "height": 4,
        "gtype": "gage",
        "title": "Traffic Density Road 2",
        "label": "%",
        "format": "{{value}}%",
        "min": 0,
        "max": 100,
        "colors": [
            "#00b500",
            "#e5e500",
            "#ca3838"
        ],
        "seg1": "",
        "seg2": "",
        "x": 920.7143859863281,
        "y": 685.714376449585,
        "wires": []
    },
    {
        "id": "e0efdc0aa87824a1",
        "type": "ui_template",
        "z": "8b886473a825886d",
        "group": "626a51c24821ce56",
        "name": "traffic_2",
        "order": 5,
        "width": 6,
        "height": 8,
        "format": "<div style=\"text-align:center;\">\n    <h2>Road 2</h2>\n    <div id=\"traffic-light-road2\"\n        style=\"width:100px; height:300px; margin:auto; background-color:#000000; padding:10px; border-radius:10px;\">\n        <div id=\"light-road2-red\"\n            style=\"background-color:red; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\">\n        </div>\n        <div id=\"light-road2-yellow\"\n            style=\"background-color:yellow; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\">\n        </div>\n        <div id=\"light-road2-green\"\n            style=\"background-color:green; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\">\n        </div>\n    </div>\n</div>\n\n<script>\n    (function(scope) {\n    scope.$watch('msg.payload', function(newValue, oldValue) {\n      if (newValue === 'green') {\n        document.getElementById('light-road2-green').style.opacity = 1;\n        document.getElementById('light-road2-yellow').style.opacity = 0.2;\n        document.getElementById('light-road2-red').style.opacity = 0.2;\n      } else if (newValue === 'yellow') {\n        document.getElementById('light-road2-green').style.opacity = 0.2;\n        document.getElementById('light-road2-yellow').style.opacity = 1;\n        document.getElementById('light-road2-red').style.opacity = 0.2;\n      } else if (newValue === 'red') {\n        document.getElementById('light-road2-green').style.opacity = 0.2;\n        document.getElementById('light-road2-yellow').style.opacity = 0.2;\n        document.getElementById('light-road2-red').style.opacity = 1;\n      }\n    });\n  })(scope);\n</script>",
        "storeOutMessages": true,
        "fwdInMessages": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "x": 2233.214385986328,
        "y": 1365.714376449585,
        "wires": [
            []
        ]
    },
    {
        "id": "cb017fb87cfc7fd3",
        "type": "ui_template",
        "z": "8b886473a825886d",
        "group": "626a51c24821ce56",
        "name": "traffic_1",
        "order": 3,
        "width": 6,
        "height": 8,
        "format": "<div style=\"text-align:center;\">\n  <h2>Road 1 </h2>\n  <div id=\"traffic-light-road1\" style=\"width:100px; height:300px; margin:auto; background-color:#000000; padding:10px; border-radius:10px;\">\n    <div id=\"light-road1-red\" style=\"background-color:red; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\"></div>\n    <div id=\"light-road1-yellow\" style=\"background-color:yellow; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\"></div>\n    <div id=\"light-road1-green\" style=\"background-color:green; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\"></div>\n  </div>\n</div>\n\n<script>\n  (function(scope) {\n    scope.$watch('msg.payload', function(newValue, oldValue) {\n      if (newValue === 'green') {\n        document.getElementById('light-road1-green').style.opacity = 1;\n        document.getElementById('light-road1-yellow').style.opacity = 0.2;\n        document.getElementById('light-road1-red').style.opacity = 0.2;\n      } else if (newValue === 'yellow') {\n        document.getElementById('light-road1-green').style.opacity = 0.2;\n        document.getElementById('light-road1-yellow').style.opacity = 1;\n        document.getElementById('light-road1-red').style.opacity = 0.2;\n      } else if (newValue === 'red') {\n        document.getElementById('light-road1-green').style.opacity = 0.2;\n        document.getElementById('light-road1-yellow').style.opacity = 0.2;\n        document.getElementById('light-road1-red').style.opacity = 1;\n      }\n    });\n  })(scope);\n</script>\n",
        "storeOutMessages": true,
        "fwdInMessages": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "x": 2233.214385986328,
        "y": 985.714376449585,
        "wires": [
            []
        ]
    },
    {
        "id": "0ce95131b0f446cd",
        "type": "ui_template",
        "z": "8b886473a825886d",
        "group": "626a51c24821ce56",
        "name": "ambulance_temp",
        "order": 20,
        "width": 6,
        "height": 6,
        "format": "<div id=\"status\"\n    style=\"text-align:center; font-size: 28px; padding: 20px; border-radius: 10px; border: 2px solid #ccc; width: 80%; margin: 20px auto; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);\">\n</div>\n\n<script>\n    (function(scope) {\n    scope.$watch('msg.payload', function(payload) {\n        var statusDiv = document.getElementById('status');\n        \n        // Reset default styles\n        statusDiv.style.backgroundColor = \"#f4f4f4\";\n        statusDiv.style.color = \"black\";\n        statusDiv.style.fontWeight = \"normal\";\n        \n        if (payload === 'r1') {\n            statusDiv.innerHTML = \"<strong>Ambulance Alert:</strong> Coming from <strong>Road 1</strong>\";\n            statusDiv.style.backgroundColor = \"#ffe5e5\"; // Light red background\n            statusDiv.style.color = \"darkred\";\n            statusDiv.style.fontWeight = \"bold\";\n        } else if (payload === 'r2') {\n            statusDiv.innerHTML = \"<strong>Ambulance Alert:</strong> Coming from <strong>Road 2</strong>\";\n            statusDiv.style.backgroundColor = \"#fff5e0\"; // Light orange background\n            statusDiv.style.color = \"darkorange\";\n            statusDiv.style.fontWeight = \"bold\";\n        } else if (payload === 'done') {\n            statusDiv.innerHTML = \"<strong>System Update:</strong> Roads are now <strong>clear</strong> and normal operation resumed.\";\n            statusDiv.style.backgroundColor = \"#e0ffe0\"; // Light green background\n            statusDiv.style.color = \"green\";\n            statusDiv.style.fontWeight = \"bold\";\n        } else {\n            statusDiv.innerHTML = \"<strong>Awaiting Data:</strong> No new information received.\";\n            statusDiv.style.backgroundColor = \"#f4f4f4\"; // Default grey background\n            statusDiv.style.color = \"grey\";\n        }\n    });\n})(scope);\n</script>",
        "storeOutMessages": true,
        "fwdInMessages": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "x": 870,
        "y": 1140,
        "wires": [
            []
        ]
    },
    {
        "id": "efe1e22b04148b84",
        "type": "ui_template",
        "z": "8b886473a825886d",
        "group": "626a51c24821ce56",
        "name": "",
        "order": 16,
        "width": 6,
        "height": 4,
        "format": "<div id=\"incident\" style=\"text-align:center; font-size: 28px; padding: 20px; border-radius: 10px; border: 2px solid #ccc; width: 80%; margin: 20px auto; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);\"></div>\n\n<script>\n(function(scope) {\n    scope.$watch('msg.payload', function(payload) {\n        var incidentDiv = document.getElementById('incident');\n        \n        // Reset default styles\n        incidentDiv.style.backgroundColor = \"#f4f4f4\";\n        incidentDiv.style.color = \"black\";\n        incidentDiv.style.fontWeight = \"normal\";\n        \n        if (payload === 'road1') {\n            incidentDiv.innerHTML = \"<strong>Accident Alert:</strong> Incident on <strong>Road 1</strong>\";\n            incidentDiv.style.backgroundColor = \"#ffe5e5\"; // Light red background\n            incidentDiv.style.color = \"darkred\";\n            incidentDiv.style.fontWeight = \"bold\";\n        } else if (payload === 'road2') {\n            incidentDiv.innerHTML = \"<strong>Accident Alert:</strong> Incident on <strong>Road 2</strong>\";\n            incidentDiv.style.backgroundColor = \"#fff5e0\"; // Light orange background\n            incidentDiv.style.color = \"darkorange\";\n            incidentDiv.style.fontWeight = \"bold\";\n        } else if (payload === 'safe') {\n            incidentDiv.innerHTML = \"<strong>Safety Update:</strong> Roads are now <strong>safe</strong>.\";\n            incidentDiv.style.backgroundColor = \"#e0ffe0\"; // Light green background\n            incidentDiv.style.color = \"green\";\n            incidentDiv.style.fontWeight = \"bold\";\n        } else {\n            incidentDiv.innerHTML = \"<strong>Monitoring Traffic:</strong> No incidents reported.\";\n            incidentDiv.style.backgroundColor = \"#f4f4f4\"; // Default grey background\n            incidentDiv.style.color = \"grey\";\n        }\n    });\n})(scope);\n</script>\n",
        "storeOutMessages": true,
        "fwdInMessages": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "x": 1660,
        "y": 1740,
        "wires": [
            []
        ]
    },
    {
        "id": "9fa8cf8280231ec7",
        "type": "mqtt in",
        "z": "8b886473a825886d",
        "name": "",
        "topic": "ACC",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "856744da1a0ab30d",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 1390,
        "y": 1740,
        "wires": [
            [
                "efe1e22b04148b84"
            ]
        ]
    },
    {
        "id": "bfec45e62ac472c7",
        "type": "ui_template",
        "z": "8b886473a825886d",
        "group": "626a51c24821ce56",
        "name": "clock",
        "order": 1,
        "width": 0,
        "height": 0,
        "format": "<div id=\"digital-clock\" class=\"clock-container\">\n  <div id=\"time\" class=\"clock-time\"></div>\n</div>\n\n<style>\n  @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@500&display=swap');\n\n  .clock-container {\n    display: flex;\n    justify-content: center;\n    align-items: center;\n    height: 100px;\n    background: linear-gradient(135deg, #1f1f1f, #333333);\n    border-radius: 10px;\n    box-shadow: 0 0 15px rgba(0, 0, 0, 0.6);\n    padding: 15px;\n  }\n\n  .clock-time {\n    font-family: 'Orbitron', sans-serif;\n    font-size: 50px;\n    background: linear-gradient(90deg, #00e0ff, #00ff94);\n    -webkit-background-clip: text;\n    color: transparent;\n    letter-spacing: 5px;\n    text-shadow: 0 0 7px rgba(0, 255, 183, 0.8), 0 0 15px rgba(0, 255, 183, 0.6), 0 0 20px rgba(0, 255, 183, 0.4);\n    animation: glow 1.5s infinite alternate;\n  }\n\n  .colon {\n    opacity: 1;\n    animation: blink 1s infinite;\n  }\n\n  @keyframes blink {\n\n    0%,\n    50% {\n      opacity: 1;\n    }\n\n    50%,\n    100% {\n      opacity: 0;\n    }\n  }\n\n  @keyframes glow {\n    from {\n      text-shadow: 0 0 7px rgba(0, 255, 183, 0.8), 0 0 15px rgba(0, 255, 183, 0.6), 0 0 20px rgba(0, 255, 183, 0.4);\n    }\n\n    to {\n      text-shadow: 0 0 15px rgba(0, 255, 183, 1), 0 0 25px rgba(0, 255, 183, 0.8), 0 0 30px rgba(0, 255, 183, 0.6);\n    }\n  }\n</style>\n\n<script>\n  function updateTime() {\n    const now = new Date();\n    const hours = String(now.getHours()).padStart(2, '0');\n    const minutes = String(now.getMinutes()).padStart(2, '0');\n    const seconds = String(now.getSeconds()).padStart(2, '0');\n\n    document.getElementById('time').innerHTML = `${hours}<span class=\"colon\">:</span>${minutes}<span class=\"colon\">:</span>${seconds}`;\n  }\n\n  setInterval(updateTime, 1000);\n  updateTime(); // Initial call to show time immediately\n</script>",
        "storeOutMessages": true,
        "fwdInMessages": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "x": 1810,
        "y": 1140,
        "wires": [
            []
        ]
    },
    {
        "id": "8b9356c892b40cba",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 2,
        "width": 7,
        "height": 1
    },
    {
        "id": "d03002e1973acc90",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 4,
        "width": 2,
        "height": 1
    },
    {
        "id": "8921868f5fefdc5a",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 6,
        "width": 7,
        "height": 1
    },
    {
        "id": "f1ca057a22215fee",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 7,
        "width": 7,
        "height": 1
    },
    {
        "id": "ce24a282cc101b0c",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 8,
        "width": 2,
        "height": 1
    },
    {
        "id": "32113d3277c03db5",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 9,
        "width": 7,
        "height": 1
    },
    {
        "id": "8896b275dbfe981b",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 10,
        "width": 7,
        "height": 1
    },
    {
        "id": "817edcabf06cbebf",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 11,
        "width": 2,
        "height": 1
    },
    {
        "id": "6b9d15a17948460a",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 12,
        "width": 7,
        "height": 1
    },
    {
        "id": "0f504d5495db86af",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 13,
        "width": 7,
        "height": 1
    },
    {
        "id": "48c0f61f0beae93b",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 14,
        "width": 2,
        "height": 1
    },
    {
        "id": "405fbf326552e2fe",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 15,
        "width": 7,
        "height": 1
    },
    {
        "id": "e075059cf3260910",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 17,
        "width": 1,
        "height": 1
    },
    {
        "id": "ea90f9519755a0e0",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 18,
        "width": 2,
        "height": 1
    },
    {
        "id": "5782088954e0ebf3",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 19,
        "width": 1,
        "height": 1
    },
    {
        "id": "2a280bb2df09f170",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 21,
        "width": 1,
        "height": 1
    },
    {
        "id": "a0185e51ca1b40dd",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 22,
        "width": 2,
        "height": 1
    },
    {
        "id": "02305add4657bb9a",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 23,
        "width": 1,
        "height": 1
    },
    {
        "id": "f3f9eec5791b46e2",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 24,
        "width": 1,
        "height": 1
    },
    {
        "id": "785ca090aa67db49",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 25,
        "width": 2,
        "height": 1
    },
    {
        "id": "86aa99fb4dd9665e",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 26,
        "width": 1,
        "height": 1
    },
    {
        "id": "8f3ff2b161eb01a7",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 27,
        "width": 1,
        "height": 1
    },
    {
        "id": "4e8f4f7f5053332b",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 28,
        "width": 2,
        "height": 1
    },
    {
        "id": "916db658f56a4521",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 29,
        "width": 1,
        "height": 1
    },
    {
        "id": "b69b9d1954538150",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 31,
        "width": 16,
        "height": 1
    },
    {
        "id": "44c4fc80a49e88bd",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 32,
        "width": 7,
        "height": 1
    },
    {
        "id": "b368608c1e09d6df",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 34,
        "width": 2,
        "height": 1
    },
    {
        "id": "f20db82baec9d31f",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 36,
        "width": 1,
        "height": 1
    },
    {
        "id": "f0be992f9964e44c",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 37,
        "width": 7,
        "height": 1
    },
    {
        "id": "3781cf8de035e1fb",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 38,
        "width": 2,
        "height": 1
    },
    {
        "id": "56e65d9fb82440ac",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 39,
        "width": 7,
        "height": 1
    },
    {
        "id": "9fc94401ebd37b08",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 40,
        "width": 7,
        "height": 1
    },
    {
        "id": "acffee17fe98cdc4",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 41,
        "width": 2,
        "height": 1
    },
    {
        "id": "8002430cb97905ba",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 42,
        "width": 7,
        "height": 1
    },
    {
        "id": "4cbbfc5ebc828507",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 43,
        "width": 7,
        "height": 1
    },
    {
        "id": "17cef47d7e0763a1",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 44,
        "width": 2,
        "height": 1
    },
    {
        "id": "a0abeee3b3fe9cfe",
        "type": "ui_spacer",
        "z": "8b886473a825886d",
        "name": "spacer",
        "group": "626a51c24821ce56",
        "order": 45,
        "width": 7,
        "height": 1
    },
    {
        "id": "856744da1a0ab30d",
        "type": "mqtt-broker",
        "name": "",
        "broker": "7ce86da1d434455a898586cc107e7e46.s1.eu.hivemq.cloud",
        "port": "8883",
        "tls": "",
        "clientid": "",
        "autoConnect": true,
        "usetls": true,
        "protocolVersion": "5",
        "keepalive": "60",
        "cleansession": true,
        "autoUnsubscribe": true,
        "birthTopic": "",
        "birthQos": "0",
        "birthRetain": "false",
        "birthPayload": "",
        "birthMsg": {},
        "closeTopic": "",
        "closeQos": "0",
        "closeRetain": "false",
        "closePayload": "",
        "closeMsg": {},
        "willTopic": "",
        "willQos": "0",
        "willRetain": "false",
        "willPayload": "",
        "willMsg": {},
        "userProps": "",
        "sessionExpiry": ""
    },
    {
        "id": "2a1b726d1c898952",
        "type": "pushbullet-config",
        "name": "osama"
    },
    {
        "id": "626a51c24821ce56",
        "type": "ui_group",
        "name": "ALL",
        "tab": "101e3f338913fdf1",
        "order": 1,
        "disp": true,
        "width": 28,
        "collapse": false
    },
    {
        "id": "101e3f338913fdf1",
        "type": "ui_tab",
        "name": "Traffic Managment System",
        "icon": "dashboard",
        "order": 3,
        "disabled": false,
        "hidden": false
    }
]