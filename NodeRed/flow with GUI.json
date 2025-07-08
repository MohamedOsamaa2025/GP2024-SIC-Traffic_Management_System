[
    {
        "id": "1ca400f5e70ab890",
        "type": "tab",
        "label": "Flow 1",
        "disabled": false,
        "info": "",
        "env": []
    },
    {
        "id": "1e7de2d8aa1ea4c3",
        "type": "mqtt out",
        "z": "1ca400f5e70ab890",
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
        "id": "92b710d7f714defe",
        "type": "change",
        "z": "1ca400f5e70ab890",
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
                "be9a0d06689dd836"
            ]
        ]
    },
    {
        "id": "ef22b842786d526a",
        "type": "mqtt out",
        "z": "1ca400f5e70ab890",
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
        "id": "3c644c9f20c064bd",
        "type": "mqtt in",
        "z": "1ca400f5e70ab890",
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
                "d804053db4d1c955"
            ]
        ]
    },
    {
        "id": "a1fadc6732de7a45",
        "type": "mqtt in",
        "z": "1ca400f5e70ab890",
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
                "a270add0940deb71"
            ]
        ]
    },
    {
        "id": "9772037f40b15490",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "d804053db4d1c955",
        "type": "function",
        "z": "1ca400f5e70ab890",
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
                "60d344f8ef2337d7",
                "fbacbba6bf57a88b"
            ]
        ]
    },
    {
        "id": "a270add0940deb71",
        "type": "function",
        "z": "1ca400f5e70ab890",
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
                "60d344f8ef2337d7",
                "181b2fcadf45f1fe"
            ]
        ]
    },
    {
        "id": "60d344f8ef2337d7",
        "type": "function",
        "z": "1ca400f5e70ab890",
        "name": "function 4",
        "func": "let value1 = flow.get('value1'); \nlet value2 = flow.get('value2');\n\n// Check if both values are available\nif (value1 !== null && value2 !== null) {\n    if (value1 > value2) {\n        msg.payload = `road1`;  // Send 'road1' if value1 is greater\n    } else if (value2 > value1) {\n        msg.payload = `road2`;  // Send 'road2' if value2 is greater\n    }\n\n// Clear the stored values\nflow.set('value1', null);\nflow.set('value2', null);\n\nreturn msg;\n\n} else {\n    // Store the incoming value if available\n    if (msg.payload.value1 !== undefined) {\n        flow.set('value1', msg.payload.value1);\n    }\n    if (msg.payload.value2 !== undefined) {\n        flow.set('value2', msg.payload.value2);\n    }\n\n    // Return null to indicate waiting for both values\n    return null;\n}\n",
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
                "8ac194ac29be87f4",
                "1ae7bfa55a2d598c",
                "2d8a7ea72bc529e8"
            ]
        ]
    },
    {
        "id": "8ac194ac29be87f4",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "a063db91e5352852",
        "type": "pushbullet in",
        "z": "1ca400f5e70ab890",
        "config": "8924e440900b2f16",
        "name": "Pushbullet Emergency",
        "x": 520.7143859863281,
        "y": 1195.714376449585,
        "wires": [
            [
                "ef22b842786d526a",
                "9772037f40b15490",
                "92b710d7f714defe",
                "76060ba10527d040"
            ]
        ]
    },
    {
        "id": "be9a0d06689dd836",
        "type": "function",
        "z": "1ca400f5e70ab890",
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
                "1e7de2d8aa1ea4c3",
                "0807cdb4e1a968bd",
                "d1ab769dc553e5c4"
            ],
            [
                "25be4424b2c68ced",
                "d409a8af302dfbe3",
                "74f45a7b733e1043"
            ]
        ]
    },
    {
        "id": "1ae7bfa55a2d598c",
        "type": "change",
        "z": "1ca400f5e70ab890",
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
                "be9a0d06689dd836",
                "bcc6fafbc748215a"
            ]
        ]
    },
    {
        "id": "2d8a7ea72bc529e8",
        "type": "mqtt out",
        "z": "1ca400f5e70ab890",
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
        "id": "05859fbdfc620808",
        "type": "ui_button",
        "z": "1ca400f5e70ab890",
        "name": "Send Alert",
        "group": "f6fb852b51275ffd",
        "order": 2,
        "width": 0,
        "height": 0,
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
                "4c756c0ee87b109b",
                "3018f933aaed7389",
                "2d8a7ea72bc529e8"
            ]
        ]
    },
    {
        "id": "4c756c0ee87b109b",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "21787b3acb673b14",
        "type": "mqtt in",
        "z": "1ca400f5e70ab890",
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
                "1a82c7374746e4fc",
                "5170b0dadc76da98"
            ]
        ]
    },
    {
        "id": "a1aa070952e28f28",
        "type": "mqtt in",
        "z": "1ca400f5e70ab890",
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
                "3f4bed838e3f45f2",
                "5170b0dadc76da98"
            ]
        ]
    },
    {
        "id": "1a82c7374746e4fc",
        "type": "function",
        "z": "1ca400f5e70ab890",
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
                "2d4567a82e9c0a0e",
                "85f513316d6ebc2b"
            ]
        ]
    },
    {
        "id": "3f4bed838e3f45f2",
        "type": "function",
        "z": "1ca400f5e70ab890",
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
                "091e7c7a243b39e1",
                "85f513316d6ebc2b"
            ]
        ]
    },
    {
        "id": "25be4424b2c68ced",
        "type": "mqtt out",
        "z": "1ca400f5e70ab890",
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
        "id": "451a2090baf8676b",
        "type": "change",
        "z": "1ca400f5e70ab890",
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
                "be9a0d06689dd836"
            ]
        ]
    },
    {
        "id": "bcc6fafbc748215a",
        "type": "switch",
        "z": "1ca400f5e70ab890",
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
        "x": 1390.7143859863281,
        "y": 1635.714376449585,
        "wires": [
            [
                "1bbfdd0ee7978d8f"
            ],
            [
                "a8ddb20d20cb8829"
            ]
        ]
    },
    {
        "id": "1bbfdd0ee7978d8f",
        "type": "pushbullet",
        "z": "1ca400f5e70ab890",
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
        "id": "a8ddb20d20cb8829",
        "type": "pushbullet",
        "z": "1ca400f5e70ab890",
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
        "id": "0807cdb4e1a968bd",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "2d4567a82e9c0a0e",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "091e7c7a243b39e1",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "d409a8af302dfbe3",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "fbacbba6bf57a88b",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "181b2fcadf45f1fe",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "85f513316d6ebc2b",
        "type": "function",
        "z": "1ca400f5e70ab890",
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
                "451a2090baf8676b",
                "c74f77610b060977"
            ]
        ]
    },
    {
        "id": "c74f77610b060977",
        "type": "debug",
        "z": "1ca400f5e70ab890",
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
        "id": "3018f933aaed7389",
        "type": "change",
        "z": "1ca400f5e70ab890",
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
                "be9a0d06689dd836"
            ]
        ]
    },
    {
        "id": "4673bebd5e761c19",
        "type": "inject",
        "z": "1ca400f5e70ab890",
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
        "y": 1655.714376449585,
        "wires": [
            [
                "a270add0940deb71"
            ]
        ]
    },
    {
        "id": "d2b374585b0a51e1",
        "type": "inject",
        "z": "1ca400f5e70ab890",
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
        "y": 1415.714376449585,
        "wires": [
            [
                "d804053db4d1c955"
            ]
        ]
    },
    {
        "id": "5170b0dadc76da98",
        "type": "function",
        "z": "1ca400f5e70ab890",
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
                "e828cb6b3d6241f5"
            ],
            [
                "084459892fabc108"
            ]
        ]
    },
    {
        "id": "e828cb6b3d6241f5",
        "type": "ui_gauge",
        "z": "1ca400f5e70ab890",
        "name": "",
        "group": "47db465fa4d59cd2",
        "order": 1,
        "width": 0,
        "height": 0,
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
        "id": "084459892fabc108",
        "type": "ui_gauge",
        "z": "1ca400f5e70ab890",
        "name": "",
        "group": "47db465fa4d59cd2",
        "order": 2,
        "width": 0,
        "height": 0,
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
        "id": "74f45a7b733e1043",
        "type": "ui_template",
        "z": "1ca400f5e70ab890",
        "group": "6ebced8ba36239c9",
        "name": "",
        "order": 1,
        "width": 0,
        "height": 0,
        "format": "<div style=\"text-align:center;\">\n    <h2>Road 2 Traffic Light</h2>\n    <div id=\"traffic-light-road2\"\n        style=\"width:100px; height:300px; margin:auto; background-color:#333; padding:10px; border-radius:10px;\">\n        <div id=\"light-road2-red\"\n            style=\"background-color:red; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\">\n        </div>\n        <div id=\"light-road2-yellow\"\n            style=\"background-color:yellow; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\">\n        </div>\n        <div id=\"light-road2-green\"\n            style=\"background-color:green; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\">\n        </div>\n    </div>\n</div>\n\n<script>\n    (function(scope) {\n    scope.$watch('msg.payload', function(newValue, oldValue) {\n      if (newValue === 'green') {\n        document.getElementById('light-road2-green').style.opacity = 1;\n        document.getElementById('light-road2-yellow').style.opacity = 0.2;\n        document.getElementById('light-road2-red').style.opacity = 0.2;\n      } else if (newValue === 'yellow') {\n        document.getElementById('light-road2-green').style.opacity = 0.2;\n        document.getElementById('light-road2-yellow').style.opacity = 1;\n        document.getElementById('light-road2-red').style.opacity = 0.2;\n      } else if (newValue === 'red') {\n        document.getElementById('light-road2-green').style.opacity = 0.2;\n        document.getElementById('light-road2-yellow').style.opacity = 0.2;\n        document.getElementById('light-road2-red').style.opacity = 1;\n      }\n    });\n  })(scope);\n</script>",
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
        "id": "d1ab769dc553e5c4",
        "type": "ui_template",
        "z": "1ca400f5e70ab890",
        "group": "d04f2ed6eb1b5b5c",
        "name": "",
        "order": 1,
        "width": 0,
        "height": 0,
        "format": "<div style=\"text-align:center;\">\n  <h2>Road 1 Traffic Light</h2>\n  <div id=\"traffic-light-road1\" style=\"width:100px; height:300px; margin:auto; background-color:#333; padding:10px; border-radius:10px;\">\n    <div id=\"light-road1-red\" style=\"background-color:red; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\"></div>\n    <div id=\"light-road1-yellow\" style=\"background-color:yellow; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\"></div>\n    <div id=\"light-road1-green\" style=\"background-color:green; width:80px; height:80px; margin:10px auto; border-radius:50%; opacity: 0.2;\"></div>\n  </div>\n</div>\n\n<script>\n  (function(scope) {\n    scope.$watch('msg.payload', function(newValue, oldValue) {\n      if (newValue === 'green') {\n        document.getElementById('light-road1-green').style.opacity = 1;\n        document.getElementById('light-road1-yellow').style.opacity = 0.2;\n        document.getElementById('light-road1-red').style.opacity = 0.2;\n      } else if (newValue === 'yellow') {\n        document.getElementById('light-road1-green').style.opacity = 0.2;\n        document.getElementById('light-road1-yellow').style.opacity = 1;\n        document.getElementById('light-road1-red').style.opacity = 0.2;\n      } else if (newValue === 'red') {\n        document.getElementById('light-road1-green').style.opacity = 0.2;\n        document.getElementById('light-road1-yellow').style.opacity = 0.2;\n        document.getElementById('light-road1-red').style.opacity = 1;\n      }\n    });\n  })(scope);\n</script>\n",
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
        "id": "76060ba10527d040",
        "type": "ui_template",
        "z": "1ca400f5e70ab890",
        "group": "611934b6116e103a",
        "name": "",
        "order": 1,
        "width": 0,
        "height": 0,
        "format": "<div id=\"status\"\n    style=\"text-align:center; font-size: 28px; padding: 20px; border-radius: 10px; border: 2px solid #ccc; width: 80%; margin: 20px auto; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);\">\n</div>\n\n<script>\n    (function(scope) {\n    scope.$watch('msg.payload', function(payload) {\n        var statusDiv = document.getElementById('status');\n        \n        // Reset default styles\n        statusDiv.style.backgroundColor = \"#f4f4f4\";\n        statusDiv.style.color = \"black\";\n        statusDiv.style.fontWeight = \"normal\";\n        \n        if (payload === 'r1') {\n            statusDiv.innerHTML = \"<strong>Ambulance Alert:</strong> Coming from <strong>Road 1</strong>\";\n            statusDiv.style.backgroundColor = \"#ffe5e5\"; // Light red background\n            statusDiv.style.color = \"darkred\";\n            statusDiv.style.fontWeight = \"bold\";\n        } else if (payload === 'r2') {\n            statusDiv.innerHTML = \"<strong>Ambulance Alert:</strong> Coming from <strong>Road 2</strong>\";\n            statusDiv.style.backgroundColor = \"#fff5e0\"; // Light orange background\n            statusDiv.style.color = \"darkorange\";\n            statusDiv.style.fontWeight = \"bold\";\n        } else if (payload === 'done') {\n            statusDiv.innerHTML = \"<strong>System Update:</strong> Roads are now <strong>clear</strong> and normal operation resumed.\";\n            statusDiv.style.backgroundColor = \"#e0ffe0\"; // Light green background\n            statusDiv.style.color = \"green\";\n            statusDiv.style.fontWeight = \"bold\";\n        } else {\n            statusDiv.innerHTML = \"<strong>Awaiting Data:</strong> No new information received.\";\n            statusDiv.style.backgroundColor = \"#f4f4f4\"; // Default grey background\n            statusDiv.style.color = \"grey\";\n        }\n    });\n})(scope);\n</script>",
        "storeOutMessages": true,
        "fwdInMessages": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "x": 840,
        "y": 1140,
        "wires": [
            []
        ]
    },
    {
        "id": "c5d866cd3edb1a90",
        "type": "ui_template",
        "z": "1ca400f5e70ab890",
        "group": "f6fb852b51275ffd",
        "name": "",
        "order": 1,
        "width": 0,
        "height": 0,
        "format": "<div id=\"incident\" style=\"text-align:center; font-size: 28px; padding: 20px; border-radius: 10px; border: 2px solid #ccc; width: 80%; margin: 20px auto; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);\"></div>\n\n<script>\n(function(scope) {\n    scope.$watch('msg.payload', function(payload) {\n        var incidentDiv = document.getElementById('incident');\n        \n        // Reset default styles\n        incidentDiv.style.backgroundColor = \"#f4f4f4\";\n        incidentDiv.style.color = \"black\";\n        incidentDiv.style.fontWeight = \"normal\";\n        \n        if (payload === 'road1') {\n            incidentDiv.innerHTML = \"<strong>Accident Alert:</strong> Incident on <strong>Road 1</strong>\";\n            incidentDiv.style.backgroundColor = \"#ffe5e5\"; // Light red background\n            incidentDiv.style.color = \"darkred\";\n            incidentDiv.style.fontWeight = \"bold\";\n        } else if (payload === 'road2') {\n            incidentDiv.innerHTML = \"<strong>Accident Alert:</strong> Incident on <strong>Road 2</strong>\";\n            incidentDiv.style.backgroundColor = \"#fff5e0\"; // Light orange background\n            incidentDiv.style.color = \"darkorange\";\n            incidentDiv.style.fontWeight = \"bold\";\n        } else if (payload === 'safe') {\n            incidentDiv.innerHTML = \"<strong>Safety Update:</strong> Roads are now <strong>safe</strong>.\";\n            incidentDiv.style.backgroundColor = \"#e0ffe0\"; // Light green background\n            incidentDiv.style.color = \"green\";\n            incidentDiv.style.fontWeight = \"bold\";\n        } else {\n            incidentDiv.innerHTML = \"<strong>Monitoring Traffic:</strong> No incidents reported.\";\n            incidentDiv.style.backgroundColor = \"#f4f4f4\"; // Default grey background\n            incidentDiv.style.color = \"grey\";\n        }\n    });\n})(scope);\n</script>\n",
        "storeOutMessages": true,
        "fwdInMessages": true,
        "resendOnRefresh": true,
        "templateScope": "local",
        "x": 1480,
        "y": 1760,
        "wires": [
            []
        ]
    },
    {
        "id": "ee9406dc8999bd63",
        "type": "mqtt in",
        "z": "1ca400f5e70ab890",
        "name": "",
        "topic": "ACC",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "856744da1a0ab30d",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 1310,
        "y": 1760,
        "wires": [
            [
                "c5d866cd3edb1a90"
            ]
        ]
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
        "id": "8924e440900b2f16",
        "type": "pushbullet-config",
        "name": "s"
    },
    {
        "id": "f6fb852b51275ffd",
        "type": "ui_group",
        "name": "Accident Detection System",
        "tab": "f32a6489be1b1939",
        "order": 1,
        "disp": true,
        "width": 6,
        "collapse": false
    },
    {
        "id": "2a1b726d1c898952",
        "type": "pushbullet-config",
        "name": "osama"
    },
    {
        "id": "47db465fa4d59cd2",
        "type": "ui_group",
        "name": "Traffic Density",
        "tab": "f32a6489be1b1939",
        "order": 4,
        "disp": true,
        "width": "6",
        "collapse": false
    },
    {
        "id": "6ebced8ba36239c9",
        "type": "ui_group",
        "name": "Traffic Lights Road 2",
        "tab": "f32a6489be1b1939",
        "order": 3,
        "disp": true,
        "width": "6",
        "collapse": false
    },
    {
        "id": "d04f2ed6eb1b5b5c",
        "type": "ui_group",
        "name": "Traffic Lights Road 1",
        "tab": "f32a6489be1b1939",
        "order": 2,
        "disp": true,
        "width": "7",
        "collapse": false
    },
    {
        "id": "611934b6116e103a",
        "type": "ui_group",
        "name": "Ambulance",
        "tab": "f32a6489be1b1939",
        "order": 5,
        "disp": true,
        "width": "6",
        "collapse": false
    },
    {
        "id": "f32a6489be1b1939",
        "type": "ui_tab",
        "name": "Traffic Management System",
        "icon": "dashboard",
        "order": 3,
        "disabled": false,
        "hidden": false
    }
]