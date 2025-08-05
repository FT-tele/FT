//---------------------------------------------VARIABLES
//const ws = new WebSocket('ws://' + window.location.hostname + ':81/');
//const ws = new WebSocket('ws://' + window.location.hostname + ':81/');
//const ws = new WebSocket('ws://' + '192.168.4.1' + '/ws');
const ws = new WebSocket('ws://' + '192.168.4.1' + ':81');
const MsgTextDecoder = new TextDecoder("utf-8");
const MsgTextEncoder = new TextEncoder();
const ChatBox = document.getElementById("chat-box");
//-------------------------------LET
let ActiveTab = 0;//= "speech-tab";//click Tab change
let ActiveUserList = -1;
let TalkingTab = 0;// click user-item change
let TalkingUser = -1;
let TalkingMsgId = -1;
let TalkingGroup = -1;
let TalkingName = "";
let SessionGroup = 0;
let MaxMsgBytes = 192;
let MyName = "";
let WebpBlob;
let zipBlob;
let lastAlert = "";
let timeoutId;
let foundMatch = false;
let sosSend = false;
let cmdSerialNumber = 0;
//const sosArray = new Uint8Array([2, 0, 0, 0, 0, 0, 0, 0, 0]);//SOS header

let CenterFreq = 915;
let BandWidth = 250;
//-------------------------------JSON
let WhisperList = [];// received new json update.  Object.assign(WhisperList, jsonData.whisperUserList);
let MeetingList = [];
let SpeechList = [];
let ConfigList = [];

let gpsJsonStr = "";
//-------------------------------file send
const reader = new FileReader();






//---------------------------------------------FUNCTION
//----------------------------------STORAGE
//-------------------------------msg 

let globalDBHandle = null;

// Open IndexedDB only once at window.load
function loadingMsgDB() {
    let request = indexedDB.open("MsgDB", 1);

    request.onupgradeneeded = function (event) {
        let db = event.target.result;

        // Create separate object stores for each format
        if (!db.objectStoreNames.contains("picture")) db.createObjectStore("picture", { keyPath: "identifierKey" });
        if (!db.objectStoreNames.contains("audio")) db.createObjectStore("audio", { keyPath: "identifierKey" });
        if (!db.objectStoreNames.contains("zip")) db.createObjectStore("zip", { keyPath: "identifierKey" });

        // Object stores for messages
        let whisperStore = db.createObjectStore("Whisper", { keyPath: "id", autoIncrement: true });
        let meetingStore = db.createObjectStore("Meeting", { keyPath: "id", autoIncrement: true });
        let speechStore = db.createObjectStore("Speech", { keyPath: "id", autoIncrement: true });

        whisperStore.createIndex("msgIdIndex", "msgId", { unique: false });
        meetingStore.createIndex("msgIdIndex", "msgId", { unique: false });
        speechStore.createIndex("msgIdIndex", "msgId", { unique: false });
    };

    request.onsuccess = function (event) {
        globalDBHandle = event.target.result; // Keep DB live throughout session
    };

    request.onerror = function (event) {
        console.error("Error loading database:", event.target.error);
    };
};

async function initMsgDB() {
    if (!globalDBHandle) {
        globalDBHandle = await loadingMsgDB();
    }
    return globalDBHandle;
}

async function storeMsg(objName, storeMsgId, dataType, metaData, isRecved) {
    try {
        let db = await initMsgDB();
        let transaction = db.transaction(objName, "readwrite");
        let store = transaction.objectStore(objName);
        let presentTimeStr = new Date().toISOString().slice(5, 19).replace("T", " ");
        let storeGroup = { Whisper: 32, Meeting: 64, Speech: 4 }[objName] || 0;


        const messageEntry = {
            msgId: storeMsgId,
            dataType: dataType,
            messageData: dataType === 2 ? MsgTextDecoder.decode(metaData) : metaData,
            isRecved: isRecved,
            timestamp: presentTimeStr
        };



        if (storeMsgId === TalkingMsgId && storeGroup === TalkingGroup) {

            let msgObject;
            if (storeGroup == 32)
                msgObject = "Whisper";
            if (storeGroup == 64)
                msgObject = "Meeting";

            //getRecordsByMsgId(msgObject, TalkingMsgId);

            let messageEntryDiv = document.createElement("div");
            messageEntryDiv.classList.add("messageEntry-item");
            const prefix = messageEntry.isRecved ? "📩" : "↗️";
            const prefixSpan = document.createElement("span");
            const messageEntrySpan = document.createElement("span");

            prefixSpan.classList.add("prefix");

            switch (Number(messageEntry.dataType)) {



                case 2: {
                    prefixSpan.textContent = `💬(${messageEntry.timestamp})${prefix}`;
                    messageEntrySpan.textContent = messageEntry.messageData;
                    messageEntryDiv.appendChild(prefixSpan);
                    messageEntryDiv.appendChild(messageEntrySpan);
                }
                    break;

                case 3: {
                    prefixSpan.textContent = `🖼️(${messageEntry.timestamp})${prefix}`;
                    let imgIdentifier = String(messageEntry.messageData);
                    readBlob(3, imgIdentifier).then(blob => {
                        let imgURL = URL.createObjectURL(blob);
                        let imgElement = document.createElement("img");
                        imgElement.src = imgURL;
                        imgElement.classList.add("picture-style"); // Apply style
                        messageEntrySpan.appendChild(prefixSpan);
                        messageEntrySpan.appendChild(imgElement);
                    }).catch(error => console.error("Error loading image:", error));

                    messageEntryDiv.appendChild(prefixSpan);

                    messageEntryDiv.appendChild(messageEntrySpan);
                    readBlob(3, imgIdentifier);
                }//
                    break;

                case 128: {
                    prefixSpan.textContent = `🎶(${messageEntry.timestamp})${prefix}`;
                    let audioIdentifier = messageEntry.messageData;

                    readBlob(4, audioIdentifier).then(blob => {
                        let audioURL = URL.createObjectURL(blob);
                        let audioElement = document.createElement("audio");
                        audioElement.src = audioURL;
                        audioElement.controls = true;
                        audioElement.classList.add("audio-style"); // Apply style
                        messageEntrySpan.appendChild(prefixSpan);
                        messageEntrySpan.appendChild(audioElement);
                    }).catch(error => console.error("Error loading audio:", error));

                    messageEntryDiv.appendChild(prefixSpan);

                    messageEntryDiv.appendChild(messageEntrySpan);
                }
                    break;

                case 5: {
                    prefixSpan.textContent = `📦(${messageEntry.timestamp})${prefix}`;
                    messageEntrySpan.textContent = zip;
                    let zipIdentifier = messageEntry.messageData;

                    readBlob(5, zipIdentifier).then(blob => {
                        let zipURL = URL.createObjectURL(blob);
                        let link = document.createElement("a");
                        link.href = zipURL;
                        link.download = "file.zip";
                        link.textContent = "Download ZIP";
                        link.classList.add("zip-style"); // Apply style

                        messageEntrySpan.appendChild(prefixSpan);
                        messageEntrySpan.appendChild(link);
                    }).catch(error => console.error("Error downloading file:", error));


                    messageEntryDiv.appendChild(prefixSpan);

                    messageEntryDiv.appendChild(messageEntrySpan);
                }
                    break;

                case 1: {

                    prefixSpan.textContent = `(${messageEntry.timestamp})🆘🆘🆘`;
                    messageEntrySpan.textContent = messageEntry.messageData;

                    messageEntryDiv.appendChild(prefixSpan);

                    messageEntryDiv.appendChild(messageEntrySpan);
                }
                    break;

                case 42: {
                    //greeting
                    let greetingIndex = messageEntry.messageData[1];
                    let greetingMsg = MsgTextDecoder.decode(messageEntry.messageData.slice(1));
                    messageEntrySpan.textContent = messageEntry.timestamp + "greeting : " + greetingMsg;
                    const button = document.createElement("button");
                    button.textContent = "confirm";
                    button.id = `btn-${messageEntry.timestamp}`;
                    if (messageEntry.isRecved)
                        messageEntrySpan.appendChild(button);
                    button.addEventListener("click", function () {
                        let userInput = prompt("Enter your confirmation:");
                        if (userInput === null) {
                            showNotification(3000, "User canceled confirmation.");
                        } else {
                            let confirmation = MsgTextEncoder.encode(userInput);
                            let confirmationArray = new Uint8Array(3 + confirmation.length);
                            confirmationArray[0] = 128;
                            confirmationArray[1] = 6;
                            confirmationArray[2] = greetingIndex;
                            confirmationArray.set(confirmation, 3);
                            ws.send(confirmationArray);
                            deleteNotify(messageEntry.timestamp);
                        }


                    });

                    messageEntryDiv.appendChild(prefixSpan);
                    messageEntryDiv.appendChild(messageEntrySpan);
                }
                    break;
                case 43: {
                    let confirmIndex = messageEntry.messageData[1];
                    let confirmIndexMsg = MsgTextDecoder.decode(messageEntry.messageData.slice(1));
                    messageEntrySpan.textContent = messageEntry.timestamp + "confirmation : " + confirmIndexMsg;
                    const button = document.createElement("button");
                    button.textContent = "complete";
                    button.id = `btn-${messageEntry.timestamp}`;
                    button.setAttribute("confirmIndex", confirmIndex);
                    if (messageEntry.isRecved)
                        messageEntrySpan.appendChild(button);
                    button.addEventListener("click", function () {
                        let userInput = prompt("Enter buddy name:");
                        if (userInput === null) {
                            //
                        } else {
                            let tunnel = MsgTextEncoder.encode(userInput);
                            let tunnelArray = new Uint8Array(3 + tunnel.length);
                            tunnelArray[0] = 128;
                            tunnelArray[1] = 7;
                            tunnelArray[2] = confirmIndex;
                            tunnelArray.set(tunnel, 3);
                            ws.send(tunnelArray);
                            deleteNotify(messageEntry.timestamp);
                        }
                    });

                    messageEntryDiv.appendChild(prefixSpan);
                    messageEntryDiv.appendChild(messageEntrySpan);

                }
                    break;

            }


            ChatBox.appendChild(messageEntryDiv);
            ChatBox.scrollTop = ChatBox.scrollHeight;

        }

        store.add(messageEntry);

        transaction.oncomplete = () => console.log(`Data stored successfully in ${objName}`);
        transaction.onerror = event => console.error(`Error storing data: ${event.target.error}`);

    } catch (error) {
        console.error(`Database error: ${error}`);
    }
}



async function deleteMsg(objName, msgId) {
    try {
        let db = await initMsgDB();
        let transaction = db.transaction(objName, "readwrite");
        let store = transaction.objectStore(objName);
        let index = store.index("msgIdIndex");

        let request = index.getAllKeys(msgId);

        return new Promise((resolve, reject) => {
            request.onsuccess = () => {
                let keys = request.result;
                if (keys.length === 0) {
                    resolve(`No records found for msgId: ${msgId}`);
                    return;
                }

                keys.forEach((key) => store.delete(key));

                transaction.oncomplete = () => resolve(`Deleted all records matching msgId: ${msgId}`);
                transaction.onerror = (event) => reject(`Error deleting records: ${event.target.error}`);
            };

            request.onerror = (event) => reject(`Error retrieving keys for deletion: ${event.target.error}`);
        });

    } catch (error) {
        return Promise.reject(`Database error: ${error}`);
    }
}

async function deleteNotify(targetTimestamp) {
    try {
        let db = await initMsgDB();
        let transaction = db.transaction("Speech", "readwrite");
        let store = transaction.objectStore("Speech");
        let index = store.index("msgIdIndex");

        let request = index.getAll(Number(0));

        return new Promise((resolve, reject) => {
            request.onsuccess = () => {
                let allNotification = request.result;
                let keysToDelete = allNotification.filter(item => item.timestamp === targetTimestamp);

                keysToDelete.forEach((item) => {
                    let deleteRequest = store.delete(item.id);

                    deleteRequest.onerror = (event) => console.error(`Error deleting id ${item.id}:`, event.target.error);
                });

                transaction.oncomplete = () => resolve(`Deleted all records matching timestamp: ${targetTimestamp}`);
                transaction.onerror = (event) => reject(`Transaction error while deleting records: ${event.target.error}`);
            };

            request.onerror = (event) => reject(`Error retrieving keys for deletion: ${event.target.error}`);
        });

    } catch (error) {
        return Promise.reject(`Database error: ${error}`);
    }
}


async function getRecordsByMsgId(storeName, msgId) {
    try {
        let db = await initMsgDB();
        let transaction = db.transaction(storeName, "readonly");
        let store = transaction.objectStore(storeName);
        let index = store.index("msgIdIndex");
        let indexRequest = index.getAll(Number(msgId));

        ChatBox.textContent = "loading msg.";

        return new Promise((resolve, reject) => {
            indexRequest.onsuccess = () => {
                let messages = indexRequest.result;
                if (!messages || messages.length === 0) {
                    ChatBox.textContent = "No messages found for this user.";
                    return resolve([]);
                }

                messages.forEach(message => {
                    let messageDiv = document.createElement("div");
                    messageDiv.classList.add("message-item");
                    const prefix = message.isRecved ? "📩" : "↗️";
                    const prefixSpan = document.createElement("span");
                    const messageSpan = document.createElement("span");
                    prefixSpan.classList.add("prefix");
                    switch (Number(message.dataType)) {



                        case 2: {
                            prefixSpan.textContent = `💬(${message.timestamp})${prefix}`;
                            messageSpan.textContent = message.messageData;
                            messageDiv.appendChild(prefixSpan);
                            messageDiv.appendChild(messageSpan);
                        }
                            break;

                        case 3: {
                            prefixSpan.textContent = `🖼️(${message.timestamp})${prefix}`;
                            let imgIdentifier = String(message.messageData);
                            readBlob(3, imgIdentifier).then(blob => {
                                let imgURL = URL.createObjectURL(blob);
                                let imgElement = document.createElement("img");
                                imgElement.src = imgURL;
                                imgElement.classList.add("picture-style"); // Apply style
                                messageSpan.appendChild(prefixSpan);
                                messageSpan.appendChild(imgElement);
                            }).catch(error => console.error("Error loading image:", error));

                            messageDiv.appendChild(prefixSpan);

                            messageDiv.appendChild(messageSpan);
                            readBlob(3, imgIdentifier);
                        }//
                            break;

                        case 128: {
                            prefixSpan.textContent = `🎶(${message.timestamp})${prefix}`;
                            let audioIdentifier = message.messageData;

                            readBlob(4, audioIdentifier).then(blob => {
                                let audioURL = URL.createObjectURL(blob);
                                let audioElement = document.createElement("audio");
                                audioElement.src = audioURL;
                                audioElement.controls = true;
                                audioElement.classList.add("audio-style"); // Apply style
                                messageSpan.appendChild(prefixSpan);
                                messageSpan.appendChild(audioElement);
                            }).catch(error => console.error("Error loading audio:", error));

                            messageDiv.appendChild(prefixSpan);

                            messageDiv.appendChild(messageSpan);
                        }
                            break;

                        case 5: {
                            prefixSpan.textContent = `📦(${message.timestamp})${prefix}`;
                            messageSpan.textContent = zip;
                            let zipIdentifier = message.messageData;

                            readBlob(5, zipIdentifier).then(blob => {
                                let zipURL = URL.createObjectURL(blob);
                                let link = document.createElement("a");
                                link.href = zipURL;
                                link.download = "file.zip";
                                link.textContent = "Download ZIP";
                                link.classList.add("zip-style"); // Apply style

                                messageSpan.appendChild(prefixSpan);
                                messageSpan.appendChild(link);
                            }).catch(error => console.error("Error downloading file:", error));


                            messageDiv.appendChild(prefixSpan);

                            messageDiv.appendChild(messageSpan);
                        }
                            break;

                        case 1: {

                            prefixSpan.textContent = `(${message.timestamp})🆘🆘🆘`;
                            messageSpan.textContent = message.messageData;

                            messageDiv.appendChild(prefixSpan);

                            messageDiv.appendChild(messageSpan);
                        }
                            break;

                        case 42: {
                            let greetingIndex = message.messageData[1];
                            let greetingMsg = MsgTextDecoder.decode(message.messageData.slice(0));
                            messageSpan.textContent = message.timestamp + "greeting : " + greetingMsg;
                            const button = document.createElement("button");
                            button.textContent = "confirm";
                            button.id = `btn-${message.timestamp}`;
                            if (message.isRecved)
                                messageSpan.appendChild(button);
                            button.addEventListener("click", function () {
                                let userInput = prompt("Enter your confirmation:");
                                if (userInput === null) {
                                    showNotification(3000, "User canceled confirmation.");
                                } else {
                                    let confirmation = MsgTextEncoder.encode(userInput);
                                    let confirmationArray = new Uint8Array(3 + confirmation.length);
                                    confirmationArray[0] = 128;
                                    confirmationArray[1] = 6;
                                    confirmationArray[2] = greetingIndex;
                                    confirmationArray.set(confirmation, 3);
                                    ws.send(confirmationArray);
                                    deleteNotify(messageEntry.timestamp);
                                }


                            });

                            messageDiv.appendChild(prefixSpan);
                            messageDiv.appendChild(messageSpan);
                        }
                            break;
                        case 43: {
                            let confirmIndex = message.messageData[1];
                            let confirmIndexMsg = MsgTextDecoder.decode(message.messageData.slice(1));
                            messageSpan.textContent = message.timestamp + "confirmation : " + confirmIndexMsg;
                            const button = document.createElement("button");
                            button.textContent = "complete";
                            button.id = `btn-${message.timestamp}`;
                            button.setAttribute("confirmIndex", confirmIndex);
                            if (message.isRecved)
                                messageSpan.appendChild(button);
                            button.addEventListener("click", function () {
                                let userInput = prompt("Enter buddy name:");
                                if (userInput === null) {
                                    //
                                } else {
                                    let tunnel = MsgTextEncoder.encode(userInput);
                                    let tunnelArray = new Uint8Array(3 + tunnel.length);
                                    tunnelArray[0] = 128;
                                    tunnelArray[1] = 7;
                                    tunnelArray[2] = confirmIndex;
                                    tunnelArray.set(tunnel, 3);
                                    ws.send(tunnelArray);
                                    deleteNotify(messageEntry.timestamp);
                                }
                            });

                            messageDiv.appendChild(prefixSpan);
                            messageDiv.appendChild(messageSpan);

                        }
                            break;

                    }

                    ChatBox.appendChild(messageDiv);
                });

                resolve(messages);
            };

            indexRequest.onerror = () => reject(`Error fetching data from ${storeName}`);
        });



    } catch (error) {
        return Promise.reject(`Database error: ${error}`);
    }
}

async function searchAlertHistory(searchStr) {
    try {
        let db = await initMsgDB();
        let transaction = db.transaction("Speech", "readonly"); // Use "Speech"
        let store = transaction.objectStore("Speech"); // Use "Speech"
        let index = store.index("msgIdIndex");
        let indexRequest = index.getAll(Number(0)); // Use msgId 
        foundMatch = false;
        return new Promise((resolve, reject) => {
            indexRequest.onsuccess = () => {
                let messages = indexRequest.result;

                messages.forEach(message => {

                    // Check if searchStr is found in message.messageData
                    if (message.messageData && message.messageData.includes(searchStr)) {

                        foundMatch = true; // Set flag to true if a match is found
                    }
                });

                if (!foundMatch) {
                    //
                }

                resolve(messages);
            };

            indexRequest.onerror = () => reject(`Error fetching data from DB`);
        });
    } catch (error) {
        console.error("Error in getRecordsByMsgId:", error);
        throw error; // Re-throw to propagate the error
    }
}

//-------------------------------render 


function showNotification(time, message) {
    const notifyElement = document.getElementById("notify");
    notifyElement.textContent = message; // Set the message dynamically
    notifyElement.style.display = "block";
    /*
        setTimeout(() => {
            notifyElement.style.display = "none";
        }, time); // Hide after 3 seconds
        */
    if (timeoutId) {
        clearTimeout(timeoutId);
    }

    timeoutId = setTimeout(() => {
        notifyElement.style.display = "none";
    }, time);
}




let busyNotify = 0; // Initial busyNotify value


const busySignButton = document.getElementById("SOS-btn");

function updateBorderColor() {
    // Define colors for min and max
    const minColor = [0, 255, 0]; // Green (RGB)
    const maxColor = [255, 0, 0]; // Red (RGB)

    // Clamp busyNotify between 0 and 7
    const clampedValue = Math.min(Math.max(busyNotify, 0), 7);

    // Calculate interpolation ratio
    const ratio = clampedValue / 7;

    // Compute new color
    const newColor = minColor.map((min, i) =>
        Math.round(min + ratio * (maxColor[i] - min))
    );

    busySignButton.style.backgroundColor = `rgb(${newColor.join(",")})`;
}




function busyNotifyUpdate() {
    busyNotify = Math.max(0, busyNotify - 2); // Decrease by 2 every second

    setTimeout(updateBorderColor, 1000);

}

busyNotifyUpdate();






function switchTab(tabId) {
    //if (tabId)
    document.getElementById("whisper-tab").classList.remove("active");
    document.getElementById("meeting-tab").classList.remove("active");
    document.getElementById("speech-tab").classList.remove("active");
    document.getElementById(tabId).classList.add("active");
    switch (tabId) {
        case "whisper-tab":
            ActiveTab = 32;
            break;

        case "meeting-tab":
            ActiveTab = 64;
            break;
        case "speech-tab":
            ActiveTab = 4;
            break;
    }

}


function renderUserList(userList) {
    let container = document.getElementById("user-list");

    // Clear the container
    container.textContent = '';
    if (!userList || userList.length == 0) {
        const message = document.createElement("p");
        message.textContent = "Tap✨.";
        container.appendChild(message);
        return;
    }

    userList.forEach(user => {
        let userDiv = document.createElement("div");
        userDiv.classList.add("user-item");
        userDiv.dataset.listIdx = user.sessionId;
        userDiv.dataset.msgId = user.msgId;
        userDiv.dataset.name = user.name;
        userDiv.textContent = `  ${user.name}:${user.sessionId}`;
        container.appendChild(userDiv);
    });

}


function renderWhisperList(userList) {
    let whisperSelect = document.getElementById("whisperList");

    // Clear existing options
    whisperSelect.innerHTML = '';

    if (!userList || userList.length === 0) {
        let defaultOption = document.createElement("option");
        defaultOption.value = null;
        defaultOption.textContent = "no buddy";
        whisperSelect.appendChild(defaultOption);
        return;
    }

    userList.forEach(user => {
        let option = document.createElement("option");
        option.value = user.sessionId; // Use sessionId as value
        option.textContent = user.name; // Display user name 
        whisperSelect.appendChild(option);
    });
}


function joinMeeting(meetingType) {

    let encoder = new TextEncoder();
    let meetingText = document.getElementById("meeting").value;
    let roomByte = encoder.encode(meetingText);
    let roomByteSize = roomByte.length;

    if (roomByteSize < 29 && roomByteSize > 7) {
        if (meetingType == 'Random') {
            //let randomString = generateRandomString(Math.floor(Math.random() * 8) + 40); // Random length (30-50)        
            //document.getElementById("invitationKey").value = randomString;
            let mergedArray = new Uint8Array(35);
            mergedArray[0] = 65;
            mergedArray[1] = roomByteSize;
            mergedArray.set(roomByte, 2);
            ws.send(mergedArray);
            showNotification(3000, "✅ creating the safe meeting room!");
            closeForm('invite-form');
        };
        if (meetingType == 'join') {

            let mergedArray = new Uint8Array(80);
            let invitationKeyText = document.getElementById("invitationKey").value;
            let keyByte = encoder.encode(invitationKeyText);
            let keyByteSize = keyByte.length;

            if (keyByteSize > 15) {
                let date = new Date();
                let dateText = `${date.getFullYear()}${(date.getMonth() + 1).toString().padStart(2, '0')}${date.getDate().toString().padStart(2, '0')}`;
                let dateByte = encoder.encode(dateText);
                let dateByteSize = dateByte.length;
                mergedArray[0] = 65;
                mergedArray[1] = roomByteSize;
                mergedArray.set(roomByte, 2);
                mergedArray.set(dateByte, 2 + roomByteSize);
                mergedArray.set(keyByte, 2 + dateByteSize + roomByteSize);
                ws.send(mergedArray);
                showNotification(3000, "✅ join the temp meeting room!");
                document.getElementById("meeting").value = '';
                document.getElementById("invitationKey").value = '';

            } else {
                showNotification(3000, "❌  short key is unsafe!");
            };
        }

    } else {
        showNotification(3000, "❌  room Length wrong! 8~28 letters ");
    };


};


//-------------------------------float-form 
function closeForm(formId) {
    document.getElementById(formId).style.display = "none";
}

//-----------------SOS-btn  




function deliverSOS() {



    alertMsg = document.getElementById("alertMsg").value + '*' + gpsJsonStr;
    let alertMsgArray = MsgTextEncoder.encode(alertMsg);
    let alertMsgArrayLen = alertMsgArray.length;
    let sosArray = new Uint8Array(13 + alertMsgArrayLen);
    //alert : direction{trigger 0/ up 1/down 2/upAck 3/downAck 4},type{},dstMac_6/firstRelayMac,srcMac_6,metaData



    sosArray[0] = 2;//code
    sosArray[1] = 0;//direction
    sosArray[2] = 0;//type = from phone
    sosArray[3] = ConfigList["MAC_0"];
    sosArray[4] = ConfigList["MAC_1"];
    sosArray[5] = ConfigList["MAC_2"];
    sosArray[6] = ConfigList["MAC_3"];
    sosArray[7] = ConfigList["MAC_4"];
    sosArray[8] = ConfigList["MAC_5"];


    sosArray[9] = document.getElementById("list1").value;
    sosArray[10] = document.getElementById("list2").value;
    sosArray[11] = document.getElementById("list3").value;
    sosArray[12] = document.getElementById("list4").value;

    if (alertMsgArrayLen > 0)
        sosArray.set(alertMsgArray, 13);

    ws.send(sosArray);
    sosSend = true;
    showNotification(3000, " SOS is sent out,pls sent it 5mins later!");
}





//-----------------picture-btn  

function blurEffect(event) {
    event.preventDefault(); // Prevent default scrolling behavior

    const rect = canvas.getBoundingClientRect(); // Get updated canvas position & size
    const scaleX = canvas.width / rect.width;   // Adjust for scaled width
    const scaleY = canvas.height / rect.height; // Adjust for scaled height

    let x, y;

    if (event.type.startsWith("touch") && event.touches.length > 0) {
        const touch = event.touches[0];
        x = (touch.clientX - rect.left) * scaleX;
        y = (touch.clientY - rect.top) * scaleY;
    } else if (event.offsetX !== undefined && event.offsetY !== undefined) {
        x = event.offsetX;
        y = event.offsetY;
    } else {
        console.error("Could not determine touch or mouse position!");
        return;
    }


    ctx.filter = 'blur(5px)';
    ctx.drawImage(img, x - 40, y - 40, 80, 80);
    ctx.filter = 'none';
}



function deliverWebp() {
    if (WebpBlob) {
        if (confirm(`sending pic out 🚀`)) {

            if (TalkingTab == ActiveTab)
                sendFile(WebpBlob, SessionGroup, TalkingUser, 3);
            else
                showNotification(3000, "make sure buddy is selected ");
        }
    } else
        showNotification(3000, "convert to webp first ");

}




function deliverFile() {
    if (confirm(`sending file out 🚀`)) {

        if (TalkingTab == ActiveTab)
            sendFile(zipBlob, SessionGroup, TalkingUser, 3);
        else
            showNotification(3000, "chose  the right one");
    }


}


function generateRandomString(length) {
    // Using template literals to prevent unescaped line break issues
    let chars = `ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789!@#$%^&*()-=_+`;

    return Array.from({ length }, () => chars[Math.floor(Math.random() * chars.length)]).join('');
}

function handleSettingsForm() {
    const form = document.getElementById("settings-form");
    const configPage = document.getElementById("settings-page");
    configPage.style.display = "block";
    form.textContent = ""; // Clear previous fields

    const config = ConfigList; // Ensure config is an object
    // Dynamically generate input fields
    /*
    Object.keys(config).forEach(key => {
        // Create label element
        const label = document.createElement("label");
        label.textContent = `${key}: `; // Use textContent for safety

        let input;

        if (typeof config[key] === "boolean") {
            // Create switch (checkbox) for boolean values
            input = document.createElement("input");
            input.setAttribute("type", "checkbox");
            input.setAttribute("id", key);
            input.checked = config[key]; // Set checked status
        } else {
            // Create text input for other values
            input = document.createElement("input");
            input.setAttribute("type", "text");
            input.setAttribute("id", key);
            input.value = config[key]; // Set value safely
        }

        // Append elements to form
        label.appendChild(input);
        form.appendChild(label);
        form.appendChild(document.createElement("br"));
    });
*/

    const readOnlyMACKeys = ["MAC_0", "MAC_1", "MAC_2", "MAC_3", "MAC_4", "MAC_5"];

    const NoneDisplay = ["T"];
    Object.keys(config).forEach(key => {
        // Create label element
        const label = document.createElement("label");
        label.textContent = `${key}: `;


        let input;
        let readOnlyJSON;

        if (readOnlyMACKeys.includes(key)) {
            // Create read-only input for MAC addresses
            input = document.createElement("input");
            input.setAttribute("type", "text");
            input.setAttribute("id", key);
            input.setAttribute("readonly", true);
            input.value = typeof config[key] === "number"
                ? " " + config[key].toString(16).toUpperCase()
                : config[key];
        } else if (typeof config[key] === "boolean") {
            // Create checkbox for boolean values
            input = document.createElement("input");
            input.setAttribute("type", "checkbox");
            input.setAttribute("id", key);
            input.checked = config[key];
        } else {
            // Create text input for other values
            input = document.createElement("input");
            input.setAttribute("type", "text");
            input.setAttribute("id", key);
            input.value = config[key];
        }

        // Append elements to form
        label.appendChild(input);
        form.appendChild(label);
        form.appendChild(document.createElement("br"));
    });

    //renderWhisperList(MeetingList);


    // Create button container to properly structure buttons
    const buttonContainer = document.createElement("div");
    buttonContainer.style.marginTop = "10px";

    // Add Save button
    const saveButton = document.createElement("button");
    saveButton.textContent = "Save💾";
    saveButton.onclick = function (event) {
        event.preventDefault(); // Prevent form submission
        const updatedConfig = {};
        document.querySelectorAll("#settings-form input").forEach(input => {
            //updatedConfig[input.id] = input.value;
            updatedConfig[input.id] = input.type === "checkbox" ? input.checked : input.value;
        });
        MyName = updatedConfig["MyName"];
        //console.log("MyName now is" + MyName);

        if (confirm("if SavingConfig = 1 ,settings will be saved forever.WIFI related items need reboot?")) {
            ws.send(JSON.stringify(updatedConfig));
            showNotification(3000, "Settings saved!");
        }

        // Hide the form after saving
        configPage.style.display = "none";
    };

    // Add Cancel button
    let exportFlag = false;
    const cancelButton = document.createElement("button");
    cancelButton.textContent = "✕Cancel";
    cancelButton.style.marginLeft = "10px"; // Adds some space between buttons
    cancelButton.onclick = function () {
        event.preventDefault(event); // Prevent form submission
        configPage.style.display = "none";
    };

    const rebootButton = document.createElement("button");
    rebootButton.textContent = "🔄Reboot";
    rebootButton.style.marginLeft = "20px"; // Adds some space between buttons
    rebootButton.onclick = function () {
        event.preventDefault(event); // Prevent form submission
        if (confirm("Are you sure you want to reboot?")) {
            let rebootMsg = '{"T": 0,"cmdType":1}';
            ws.send(rebootMsg);
            showNotification(3000, " refresh this tab manually in  20 second  !");
            configPage.style.display = "none";
        };


    };
    const resetButton = document.createElement("button");
    resetButton.textContent = "RESET";
    resetButton.style.marginLeft = "20px"; // Adds some space between buttons
    resetButton.onclick = function () {
        event.preventDefault(event); // Prevent form submission
        if (exportFlag == true) {
            if (confirm("🚫WARNING:This action is IRREVERSIBLE.Exporting your contact first")) {
                let resetMsg = '{"T": 0,"cmdType":2}';
                ws.send(resetMsg);
                showNotification(3000, "   if browser keep history, Chat history should be  cleared manually   !");
                configPage.style.display = "none";
            };


        } else {
            if (confirm("🚫WARNING:This action is IRREVERSIBLE. system would be erased as new.")) {
                buttonContainer.appendChild(exportButton);
                exportFlag = true;
            }
        };
    };
    const exportButton = document.createElement("button");
    exportButton.textContent = "export🔐";
    exportButton.style.marginLeft = "20px"; // Adds some space between buttons
    exportButton.onclick = function () {
        event.preventDefault(event); // Prevent form submission
        if (confirm("⚠️Export settings and contact may lead privacy leak ?")) {
            let exportMsg = '{"T": 0,"cmdType":3}';
            ws.send(exportMsg);
        };

    };
    const importButton = document.createElement("button");
    importButton.textContent = "IMPORT↩";
    importButton.style.marginLeft = "20px"; // Adds some space between buttons
    importButton.onclick = function () {
        event.preventDefault(event); // Prevent form submission
        if (confirm("Are you sure you want to import your data?")) {
            //let importMsg = file.read();
            //ws.send(importMsg);
            showNotification(3000, " refresh this tab manually in  20 second  !");
            configPage.style.display = "none";
        };


    };
    // Append buttons to the container, then add it to the form
    buttonContainer.appendChild(saveButton);
    buttonContainer.appendChild(cancelButton);
    buttonContainer.appendChild(rebootButton);
    buttonContainer.appendChild(importButton);
    buttonContainer.appendChild(resetButton);

    form.appendChild(buttonContainer);
};

function addMember() {
    const clientIvm = document.getElementById("whisperList");
    const clientIvmValue = clientIvm.value;
    const clientIvmText = clientIvm.options[clientIvmValue].text;

    if (confirm("inviting " + clientIvmText + "join" + TalkingName)) {
        let ivmArray = new Uint8Array(3);
        ivmArray[0] = 128;
        ivmArray[1] = 9;
        ivmArray[2] = clientIvmValue;
        ivmArray[3] = TalkingUser;
        ws.send(ivmArray);
        let ivmNotify = MyName + "  inviting " + clientIvmText + "  join room";
        let ivmNotifyArray = MsgTextEncoder.encode(ivmNotify);
        let msgArrayLen = ivmNotifyArray.length;
        let sendMsgArray = new Uint8Array(4 + msgArrayLen);
        sendMsgArray.set(ivmNotifyArray, 4 + msgArrayLen);
        sendMsgArray[0] = 128;
        sendMsgArray[1] = TalkingUser;
        sendMsgArray[2] = 2; // dataType 
        //ws.send(sendMsgArray);--------------------------------unfinish
    };
}

//-----------------sending file


function sendFile(blob, byte0, byte1, byte2) {
    let fileSize = blob.size;
    if (fileSize > 45 * 1024) { // Enforce 50KB file size limit
        console.error("File exceeds 50KB limit.");
        showNotification(3000, "File size must be 50KB or less!");
        return;
    }

    const { progressFill, progressText, progressBarContainer } = createProgressBar();


    reader.readAsArrayBuffer(blob);
    const fileIdentifier = new Uint8Array(4);
    crypto.getRandomValues(fileIdentifier); // Generate unique identifier per file


    let identifierKey = fileIdentifier.join("-"); // Unique file ID
    if (byte0 == 32) {
        storeMsg("Whisper", WhisperList[byte1].msgId, byte2, identifierKey, false);
    }
    if (byte0 == 64) {
        storeMsg("Meeting", MeetingList[byte1].msgId, byte2, identifierKey, false);
    }

    if (fileSize > 5 * 1024) {
        let sendMsgArray = new Uint8Array(4);
        sendMsgArray[0] = byte0;
        sendMsgArray[1] = TalkingUser;
        sendMsgArray[2] = 6; // dataType
        sendMsgArray[3] = 1;
        ws.send(sendMsgArray);
        //setTimeout(() => showNotification(3000,"Receivers have been notified,start transmission "), 1000);
    }

    let objectStoreName = getObjectStoreName(byte2);
    let transaction = globalDBHandle.transaction([objectStoreName], "readwrite");
    let store = transaction.objectStore(objectStoreName);

    store.put({
        identifierKey,
        format: getMimeType(byte2),
        type: byte0,
        completeFile: true,
        completeFileBlob: new Blob([blob], { type: blob.type }),
        size: Blob.size,
    });

    reader.onload = () => {
        const uint8Array = new Uint8Array(reader.result);
        const chunkSize = 180;
        const totalChunks = Math.ceil(uint8Array.length / chunkSize);

        function sendChunk(index) {
            if (index >= totalChunks) {
                // Send End Chunk Notification
                let endChunk = new Uint8Array(8);
                endChunk[0] = byte0;
                endChunk[1] = byte1;
                endChunk[2] = byte2;
                endChunk[3] = 255; // End marker
                endChunk.set(fileIdentifier, 4);
                ws.send(endChunk);
                if (fileSize > 5 * 1024) {
                    let sendMsgArray = new Uint8Array(1);
                    sendMsgArray[0] = totalChunks;
                    ws.send(sendMsgArray);
                }

                // Remove progress bar after completion
                document.body.removeChild(progressBarContainer);
                return;
            }

            let start = index * chunkSize;
            let end = Math.min(start + chunkSize, uint8Array.length);
            let chunk = uint8Array.slice(start, end);

            let sendMsgArray = new Uint8Array(8 + chunk.length);
            sendMsgArray[0] = byte0;
            sendMsgArray[1] = byte1;
            sendMsgArray[2] = byte2;
            sendMsgArray[3] = index; // Serial number
            sendMsgArray.set(fileIdentifier, 4);
            sendMsgArray.set(chunk, 8);

            setTimeout(() => sendChunk(index + 1), 200); // Delay between sends
            ws.send(sendMsgArray);

            // Update progress bar
            let progress = ((index + 1) / totalChunks) * 100;
            progressFill.style.width = `${progress}%`;
            progressText.innerText = `Uploading: ${Math.round(progress)}%`;
        }

        sendChunk(0); // Start chunk transmission
    };




    reader.onerror = () => console.error("Error reading file.");
}

function createProgressBar() {
    let progressBarContainer = document.createElement("div");
    progressBarContainer.className = "progress-container";

    let progressText = document.createElement("p");
    progressText.innerText = "Uploading...";

    let progressBar = document.createElement("div");
    progressBar.className = "progress-bar";

    let progressFill = document.createElement("div");
    progressFill.className = "progress-fill";

    progressBar.appendChild(progressFill);
    progressBarContainer.appendChild(progressText);
    progressBarContainer.appendChild(progressBar);
    document.body.appendChild(progressBarContainer);

    return { progressFill, progressText, progressBarContainer };
}



//-----------------receiving file
// Object to manage incoming files
const activeFiles = new Map();
const timers = new Map();

function processReceivedData(data) {
    showNotification(100, "⌛");
    let byte0 = data[0]; // Session type (Whisper or Meeting)
    let byte1 = data[1];
    let byte2 = data[2]; // File format (3 = WebP, 4 = WAV, 5 = ZIP)
    let chunkIndex = data[3]; // Chunk sequence index
    let identifierKey = data.slice(4, 8).join("-"); // Unique file ID
    let chunkData = data.slice(8); // File chunk


    let objectStoreName = getObjectStoreName(byte2);

    if (!activeFiles.has(identifierKey)) {
        activeFiles.set(identifierKey, { chunks: [], format: byte2, type: byte0, receivedTime: Date.now() });
    }

    activeFiles.get(identifierKey).chunks.push({ index: chunkIndex, data: chunkData });

    if (!timers.has(identifierKey)) {
        timers.set(identifierKey, setTimeout(() => finalizeFile(identifierKey, objectStoreName, byte0, byte1, byte2), 60000));
    }
    showNotification(100, "⏳");
    if (chunkIndex === 255) {
        clearTimeout(timers.get(identifierKey));
        finalizeFile(identifierKey, objectStoreName, byte0, byte1, byte2);
    }
}

// Store the file in IndexedDB
function finalizeFile(identifierKey, objectStoreName, rcvCode, userIdx, dataType) {
    let fileData = activeFiles.get(identifierKey);
    if (!fileData || !globalDBHandle) return;

    fileData.chunks.sort((a, b) => a.index - b.index);
    let receivedIndices = fileData.chunks.map(chunk => chunk.index);
    let expectedChunks = Math.max(...receivedIndices);
    let isComplete = receivedIndices.length === expectedChunks;

    let completeFileBlob = new Blob(fileData.chunks.map(chunk => chunk.data), { type: getMimeType(fileData.format) });

    let transaction = globalDBHandle.transaction([objectStoreName], "readwrite");
    let store = transaction.objectStore(objectStoreName);

    if (rcvCode == 32) {
        storeMsg("Whisper", WhisperList[userIdx].msgId, dataType, identifierKey, true);
    }
    if (rcvCode == 64) {
        storeMsg("Meeting", MeetingList[userIdx].msgId, dataType, identifierKey, true);
    }


    store.put({
        identifierKey,
        format: fileData.format,
        type: fileData.type,
        completeFile: isComplete,
        completeFileBlob,
        size: completeFileBlob.size,
    });

    activeFiles.delete(identifierKey);
    showNotification(1000, "📦");
}

// Get correct object store name based on format
function getObjectStoreName(format) {
    return { 3: "picture", 4: "audio", 5: "zip" }[format] || "unknown";
}

// Get MIME type
function getMimeType(format) {
    return { 3: "image/webp", 4: "audio/wav", 5: "application/zip" }[format] || "application/octet-stream";
}

function readBlob(format, identifierKey) {
    return new Promise((resolve, reject) => {
        if (!globalDBHandle) {
            reject("Database is not initialized.");
            return;
        }

        let objectStoreName = getObjectStoreName(format);
        let transaction = globalDBHandle.transaction([objectStoreName], "readonly");
        let store = transaction.objectStore(objectStoreName);
        let request = store.get(identifierKey);

        request.onsuccess = function (event) {
            let fileData = event.target.result;
            if (!fileData) {
                reject(`No file found with identifier: ${identifierKey}`);
                return;
            }

            resolve(fileData.completeFileBlob);
        };

        request.onerror = function () {
            reject("Error reading from database.");
        };
    });
}


//------------------------------------------------------------------------------------------------------------phone call


// AudioWorklet processor
const workletCode = `
      class AudioProcessor extends AudioWorkletProcessor {
        constructor() {
          super();
          this.sampleBuffer = [];
          this.frameSize = 2880; // 60ms @ 48kHz
//-------------------------------------- noiseThreshold
     this.noiseThreshold = 0.01; // Initial noise gate threshold
          this.port.onmessage = (event) => {
            if (event.data.type === 'setThreshold') {
              this.noiseThreshold = event.data.value;
            }
          };
//-------------------------------------- noiseThreshold

          
        }
        process(inputs, outputs, parameters) {
          const input = inputs[0][0];
          if (input && input.length > 0) {
            const maxAmplitude = Math.max(...input);
            this.port.postMessage({ type: 'amplitude', value: maxAmplitude });
            const int16Samples = new Int16Array(input.length);
            for (let i = 0; i < input.length; i++) {
              int16Samples[i] = Math.max(-32768, Math.min(32767, input[i] * 32768));
            }
            this.sampleBuffer.push(...int16Samples);
            while (this.sampleBuffer.length >= this.frameSize) {
              const frame = this.sampleBuffer.splice(0, this.frameSize);
              this.port.postMessage({ type: 'frame', data: new Int16Array(frame) });
            }
          }
          return true;
        }
      }
      registerProcessor('audio-processor', AudioProcessor);
    `;



const FRAME_SIZE = 2880; // 60ms @ 48kHz
const GAIN = 0.8;
const HPF_FREQ = 250;
const startSpeak = document.getElementById("startSpeak");
const stopSpeak = document.getElementById("stopSpeak");
//const amplitudeDisplay = document.getElementById("amplitude");
const chunkDisplay = document.getElementById("chunkSize");
const statusDisplay = document.getElementById("status");
let stopEncoding = null;
let globalEncoder = null;

startSpeak.onclick = async () => {
    if (stopEncoding) {
        await stopEncoding();

        document.getElementById("startSpeak").textContent = "📲Speak";

    } else {

        stopEncoding = await startVoice();
        document.getElementById("startSpeak").textContent = "📢Talking";
    }

};


async function startVoice() {
    try {
        const stream = await navigator.mediaDevices.getUserMedia({
            audio: {
                channelCount: 1,
                sampleRate: 48000,
                echoCancellation: true, // Enable echo cancellation
                noiseSuppression: true // Enable noise suppression 
            }
        });
        const audioContext = new AudioContext({ sampleRate: 48000 });

        // Load AudioWorklet processor
        const blob = new Blob([workletCode], { type: 'application/javascript' });
        const url = URL.createObjectURL(blob);
        await audioContext.audioWorklet.addModule(url);

        const source = audioContext.createMediaStreamSource(stream);
        const filter = audioContext.createBiquadFilter();
        filter.type = "highpass";
        filter.frequency.value = HPF_FREQ;
        const gainNode = audioContext.createGain();
        gainNode.gain.value = GAIN;
        const processor = new AudioWorkletNode(audioContext, 'audio-processor');

        source.connect(filter);
        filter.connect(gainNode);
        gainNode.connect(processor);

        // Silent output to activate microphone
        const silentGain = audioContext.createGain();
        silentGain.gain.setValueAtTime(0, audioContext.currentTime);
        processor.connect(silentGain);
        silentGain.connect(audioContext.destination);

        let chunkCount = 0;
        let previousChunk = null;

        globalEncoder = new AudioEncoder({
            output: (chunk) => {
                if (globalEncoder?.state === "configured") {
                    const buffer = new Uint8Array(chunk.byteLength);
                    chunk.copyTo(buffer);
                    chunkCount++;
                    chunkDisplay.textContent = buffer.length;
                    statusDisplay.innerHTML = `<span class="success">Encoded chunk ${chunkCount}: ${buffer.length} bytes</span>`;

                    if (buffer.length > 50) {
                        if (!previousChunk) {
                            previousChunk = buffer;
                            return;
                        }
                        const len1 = previousChunk.length;
                        const len2 = buffer.length;
                        const combo = new Uint8Array(3 + len1 + len2);
                        combo[0] = 1;
                        combo[1] = len1;
                        combo[2] = len2;
                        combo.set(previousChunk, 3);
                        combo.set(buffer, 3 + len1);
                        if (ws.readyState === WebSocket.OPEN) {
                            ws.send(combo);
                        } else {
                            console.warn("WebSocket not open, chunk not sent");
                            statusDisplay.innerHTML = `<span class="error">WebSocket not open</span>`;
                        }
                        previousChunk = null;
                    } else {
                        console.warn(`Chunk ${chunkCount} too small: ${buffer.length} bytes`);
                    }
                }
            },
            error: (e) => {
                console.error("Encoder error:", e);
                statusDisplay.innerHTML = `<span class="error">Encoder error: ${e.message}</span>`;
            }
        });

        await globalEncoder.configure({
            codec: "opus",
            sampleRate: 48000,
            numberOfChannels: 1,
            bitrate: 8000,
            opus: { application: "voip", frameDuration: 60000 }
        });

        processor.port.onmessage = (event) => {
            if (event.data.type === 'amplitude') {
                //amplitudeDisplay.textContent = event.data.value.toFixed(4);
                if (event.data.value < 0.001) {
                    console.warn("Warning: Microphone input is silent");
                    statusDisplay.innerHTML = `<span class="error">Microphone input is silent</span>`;
                }
            } else if (event.data.type === 'frame') {
                const audioData = new AudioData({
                    format: "s16",
                    sampleRate: 48000,
                    numberOfFrames: FRAME_SIZE,
                    numberOfChannels: 1,
                    timestamp: performance.now() * 1000,
                    data: event.data.data
                });
                try {
                    globalEncoder.encode(audioData);
                } catch (e) {
                    console.error("Encode failed:", e);
                    statusDisplay.innerHTML = `<span class="error">Encode failed: ${e.message}</span>`;
                }
            }
        };

        ws.onopen = () => {
            statusDisplay.textContent = 'Connected';
        };
        ws.onclose = () => {
            statusDisplay.textContent = 'Disconnected, reconnecting...';
            setTimeout(() => { ws = new WebSocket('ws://192.168.4.1:81'); }, 1000);
        };
        ws.onerror = (e) => {
            console.error('WebSocket error:', e);
            statusDisplay.innerHTML = `<span class="error">WebSocket error</span>`;
        };

        return async () => {
            processor.disconnect();
            gainNode.disconnect();
            filter.disconnect();
            source.disconnect();
            silentGain.disconnect();
            stream.getTracks().forEach(t => t.stop());
            if (globalEncoder?.state === "configured") {
                await globalEncoder.flush();
                globalEncoder.close();
            }
            globalEncoder = null;
            await audioContext.close();
        };
    } catch (e) {
        console.error("Start voice failed:", e);
        throw e;
    }
}

//--------------------------------------------------------------------------------------------listen module

//const ws = new WebSocket('ws://192.168.4.1:81');
const sampleRate = 48000;
const channels = 1;
const frameDurationMs = 60;
const frameSamples = sampleRate * (frameDurationMs / 1000);
const audioContext = new AudioContext({ sampleRate });
let decoder = null;
let lastPlaybackTime = 0;


let frameIndex = 0;



function playPCM(pcmData) {
    const amplifiedPCM = new Float32Array(pcmData.length);
    for (let i = 0; i < pcmData.length; i++) {
        amplifiedPCM[i] = pcmData[i] * 20; // Amplify 20x
    }

    const audioBuffer = audioContext.createBuffer(channels, amplifiedPCM.length, sampleRate);
    audioBuffer.copyToChannel(amplifiedPCM, 0);
    const source = audioContext.createBufferSource();
    const gainNode = audioContext.createGain();
    gainNode.gain.setValueAtTime(1.0, audioContext.currentTime);
    source.buffer = audioBuffer;
    source.connect(gainNode);
    gainNode.connect(audioContext.destination);

    const maxAmplitude = Math.max(...pcmData);
    const minAmplitude = Math.min(...pcmData);

    const currentTime = audioContext.currentTime;
    const scheduledTime = Math.max(lastPlaybackTime, currentTime + 0.01);
    source.start(scheduledTime);
    lastPlaybackTime = scheduledTime + frameDurationMs / 1000;
}

function playTestTone() {
    const pcmData = new Float32Array(frameSamples);
    for (let i = 0; i < frameSamples; i++) {
        pcmData[i] = Math.sin(2 * Math.PI * 440 * i / sampleRate) * 1.0;
    }
    playPCM(pcmData);
    document.getElementById('status').textContent = 'Test tone played';
}

async function initializeWebSocketAndDecoder() {
    try {



        decoder = new AudioDecoder({
            output: (audioData) => {
                const pcm = new Float32Array(audioData.numberOfFrames);
                audioData.copyTo(pcm, { planeIndex: 0, format: 'f32' });
                playPCM(pcm);
                audioData.close();
            },
            error: (e) => console.error('Decoding error:', e),
        });

        await decoder.configure({
            codec: 'opus',
            sampleRate: 48000,
            numberOfChannels: 1,
        });
        // Alternative config for testing (uncomment to try):
        // await decoder.configure({
        //     codec: 'opus',
        //     sampleRate: 16000,
        //     numberOfChannels: 1,
        // });


    } catch (e) {
        console.error('Initialization error:', e);
        document.getElementById('status').textContent = 'Initialization error';
    }
}



document.getElementById('testToneBtn').addEventListener('click', () => {
    playTestTone();
});

window.addEventListener('unload', async () => {
    if (decoder) decoder.close();
    if (ws && ws.readyState === WebSocket.OPEN) ws.close();
    await audioContext.close();
});



//------------------------------------------------------------------------------------------------------------phone call






//------------------------------------------------------------------------------------------------------------phone call



//---------------------------------------------EventListener
//-----------------tab & list & button  

document.getElementById("whisper-tab").addEventListener("click", function () {
    switchTab("whisper-tab");
    document.getElementById("pic-btn").style.display = "block";
    document.getElementById("invite-btn").style.display = "block";
    document.getElementById("dial-btn").style.display = "block";
    document.getElementById("file-btn").style.display = "block";
    document.getElementById("addMeb-btn").style.display = "none";
    document.getElementById("GPS-btn").style.display = "none";
    document.getElementById("Search-btn").style.display = "none";
    document.getElementById("IoT-btn").style.display = "none";
    renderUserList(WhisperList);
});
document.getElementById("meeting-tab").addEventListener("click", function () {
    switchTab("meeting-tab");
    document.getElementById("pic-btn").style.display = "block";
    document.getElementById("invite-btn").style.display = "block";
    document.getElementById("GPS-btn").style.display = "block";
    document.getElementById("addMeb-btn").style.display = "block";
    document.getElementById("file-btn").style.display = "none";
    document.getElementById("dial-btn").style.display = "none";
    document.getElementById("Search-btn").style.display = "none";
    document.getElementById("IoT-btn").style.display = "none";
    renderUserList(MeetingList);
});
document.getElementById("speech-tab").addEventListener("click", function () {
    switchTab("speech-tab");
    document.getElementById("pic-btn").style.display = "block";
    document.getElementById("invite-btn").style.display = "none";
    document.getElementById("dial-btn").style.display = "none";
    document.getElementById("GPS-btn").style.display = "none";
    document.getElementById("file-btn").style.display = "none";
    document.getElementById("addMeb-btn").style.display = "none";
    document.getElementById("Search-btn").style.display = "block";
    document.getElementById("IoT-btn").style.display = "block";



    renderUserList(SpeechList);
});


document.getElementById("dial-btn").addEventListener("click", function () {
    document.getElementById("phone-form").style.display = "block";
})



const userList = document.getElementById("user-list");

let clickTimeout;
userList.addEventListener("click", (event) => {
    event.preventDefault();

    if (!event.target.classList.contains("user-item")) return;

    clickTimeout = setTimeout(() => {
        let talkName = document.getElementById("del-btn");
        TalkingTab = ActiveTab;
        TalkingMsgId = Number(event.target.dataset.msgId);
        TalkingUser = Number(event.target.dataset.listIdx);
        talkName.textContent = "♻️" + event.target.dataset.name;

        let groupDomId = document.getElementById("leave-btn");
        groupDomId.textContent = "❌ " + event.target.dataset.listIdx;


        TalkingName = event.target.dataset.name;
        SessionGroup = TalkingTab;
        switch (TalkingTab) {
            case 32:
                getRecordsByMsgId("Whisper", TalkingMsgId);
                break;
            case 64:
                getRecordsByMsgId("Meeting", TalkingMsgId);
                break;
            case 4:
                getRecordsByMsgId("Speech", TalkingMsgId);
                break;
        }
        TalkingGroup = SessionGroup;
        // ChatBox.scrollTop = ChatBox.scrollHeight;
    }, 300);
    setTimeout(() => {
        ChatBox.append("\n");
        ChatBox.scrollTop = ChatBox.scrollHeight;
    }, 500); // Gives extra time for UI changes

});

// Attach the `dblclick` event **once** globally
userList.addEventListener("dblclick", (event) => {
    clearTimeout(clickTimeout); // Cancels single-click action
    TalkingTab = ActiveTab;

    let nickNewInput = prompt("Enter buddy new Nickname:");

    let nickNew = nickNewInput.replace(/[^\p{L}\p{N}\s]/gu, "");
    let nickNewLen = MsgTextEncoder.encode(nickNew);
    if (nickNewLen.length > 28)
        showNotification(3000, "nick name too long");
    else {
        if (nickNew) {
            let nickJson = {
                T: 3,
                sessionIndex: TalkingUser,
                nickName: nickNew,

            };
            if (TalkingTab == 32) nickJson.sessionGroup = 0;
            if (TalkingTab == 64) nickJson.sessionGroup = 1;
            if (nickJson.sessionIndex >= 0)
                ws.send(JSON.stringify(nickJson));
            else
                showNotification(3000, "didn't pick the name up");
        } else {
            showNotification(3000, "Nickname input was cancelled.");
        }
    }

});




document.getElementById("send-btn").addEventListener("click", function () {
    const inputBox = document.getElementById("input-box");
    let messageData = MsgTextEncoder.encode(inputBox.value);
    if (messageData.length > 0 && TalkingTab == ActiveTab) {
        if (messageData.length > MaxMsgBytes) {
            showNotification(3000, "Input exceeds 200 bytes!");
            inputField.value = inputField.value.slice(0, MaxMsgBytes);
        }


        switch (SessionGroup) {
            case 32:
                {
                    let sendMsgArray = new Uint8Array(3 + messageData.length);
                    sendMsgArray[0] = 32;
                    sendMsgArray[1] = TalkingUser;
                    sendMsgArray[2] = 2; // dataType
                    sendMsgArray.set(messageData, 3);
                    ws.send(sendMsgArray);
                    storeMsg("Whisper", TalkingMsgId, 2, messageData, false)
                }

                break;
            case 64:
                {
                    let speakerPrefix = MyName + ":";
                    let speakerPrefixArray = MsgTextEncoder.encode(speakerPrefix);
                    let msgArrayLen = speakerPrefixArray.length;
                    //let sendMsgArray = new Uint8Array(3 + msgArrayLen + messageData.length);
                    let sendMsgArray = new Uint8Array(3 + msgArrayLen + messageData.length);
                    sendMsgArray[0] = 64;
                    sendMsgArray[1] = TalkingUser;
                    sendMsgArray[2] = 2; // dataType

                    sendMsgArray.set(speakerPrefixArray, 3);
                    sendMsgArray.set(messageData, 3 + msgArrayLen);
                    ws.send(sendMsgArray);
                    storeMsg("Meeting", TalkingMsgId, 2, messageData, false)
                }

                break;
            case 4:
                {
                    if (TalkingUser > 0 || sosSend == true) {
                        if (messageData.length > 10) {
                            //console.log("MyName in speech " + MyName);
                            let speakerPrefix = MyName + ":";
                            let speakerPrefixArray = MsgTextEncoder.encode(speakerPrefix);
                            let msgArrayLen = speakerPrefixArray.length;
                            let sendMsgArray = new Uint8Array(3 + msgArrayLen + messageData.length);
                            sendMsgArray[0] = 4;
                            sendMsgArray[1] = TalkingUser;
                            sendMsgArray[2] = 2; // dataType
                            sendMsgArray.set(speakerPrefixArray, 3);
                            sendMsgArray.set(messageData, 3 + msgArrayLen);
                            ws.send(sendMsgArray);
                            storeMsg("Speech", TalkingMsgId, 2, messageData, false)
                        } else {
                            showNotification(3000, "short message in public  is  not welcome  as nonesense ");
                        }

                    } else {

                        showNotification(3000, "notification only support receiving");
                    }
                }

                break;
        }

        inputBox.value = '';
    }
    else {
        showNotification(3000, "check your input and buddy");
    };
});


document.getElementById("input-box").addEventListener("keydown", function (event) {
    if (event.key === "Enter") {
        event.preventDefault(); // Prevents default behavior (e.g., new line in textarea)
        document.getElementById("send-btn").click(); // Simulates button click
    }
});


document.getElementById("invite-btn").addEventListener("click", function () {
    if (ActiveTab == 64) {
        document.getElementById("invite-form").style.display = "block"; // Show form	
    } else {
        let inviteMsg = document.getElementById("input-box");
        if (inviteMsg.value.length == 0) {
            if (confirm("Updating the key may result in the loss of session confirmation.")) {
                let updateKeyMsg = '{"T": 4,"cmdType":4}';//update key
                ws.send(updateKeyMsg);
                showNotification(3000, "key is updating");
            };

        } else {
            if (confirm("People nearby would receive your invitation;")) {
                //WEB:code,greetingMsg
                let greetingSend = inviteMsg.value;
                let greetingEncoded = MsgTextEncoder.encode(greetingSend);
                let greetingArray = new Uint8Array(greetingEncoded.length + 2);
                greetingArray[0] = 128;
                greetingArray[1] = 5;
                greetingArray.set(greetingEncoded, 2);
                ws.send(greetingArray);
                showNotification(3000, "greetingMsg is sending out!");
            }
        };
    };

});

document.getElementById("file-btn").addEventListener("click", async () => {
    const input = document.createElement("input");
    input.type = "file";
    input.style.display = "none";
    document.body.appendChild(input);

    input.addEventListener("change", async (event) => {
        const file = event.target.files[0];
        if (!file) return;

        // Read the file as ArrayBuffer
        const reader = new FileReader();
        reader.readAsArrayBuffer(file);
        reader.onload = async () => {
            const fileData = new Uint8Array(reader.result);

            // Compress using Pako
            const compressedData = pako.deflate(fileData, { level: 9 });
            const compressedBlob = new Blob([compressedData], { type: "application/octet-stream" });

            const fileSize = compressedBlob.size;
            const sendFileForm = document.getElementById("sendFile-form");

            if (fileSize < 45 * 1024) {
                zipBlob = compressedBlob;
                sendFileForm.style.display = "block";
            } else {
                sendFileForm.style.display = "none";
                showNotification(3000, "File size out of permission.");
            }

        };
    });

    input.click();
});





// Open Floating Forms
document.getElementById("pic-btn").addEventListener("click", function () {
    document.getElementById("pic-form").style.display = "block";
    document.getElementById("imageInput").click();
});

document.getElementById("Search-btn").addEventListener("click", function () {
    document.getElementById("find-form").style.display = "block";
    document.querySelector('.sos-input').style.display = 'block';
    document.querySelector('.IoT-inputs').style.display = 'none';
    //initColorPicker('colorCanvas', 'inputR', 'inputG', 'inputB', 'colorPreview');
    //console.log("open find form");
});

document.getElementById("IoT-btn").addEventListener("click", function () {
    document.getElementById("find-form").style.display = "block";
    document.querySelector('.IoT-inputs').style.display = 'block';
    document.querySelector('.sos-input').style.display = 'none';
    //console.log("open find form");
});


document.getElementById("SOS-btn").addEventListener("click", function () {
    document.getElementById("sos-form").style.display = "block";
});

document.getElementById("GPS-btn").addEventListener("click", function () {

    let oldestKey = null;
    let oldestData = null;

    // Check sessionStorage for previous location data
    for (let i = 0; i < sessionStorage.length; i++) {
        const key = sessionStorage.key(i);
        if (!oldestKey || new Date(key) < new Date(oldestKey)) {
            oldestKey = key;
            oldestData = JSON.parse(sessionStorage.getItem(key));
        }
    }





    document.getElementById("sensor-form").style.display = "block";
})

function uploadKey(file) {
    if (!file) {
        console.error("No file provided");
        return;
    }
    const reader = new FileReader();

    reader.onload = function (event) {
        const fileContent = event.target.result; // This will be a string

        const jsonObj = {
            T: 6,
            sensorGroupId: TalkingUser,
            sensorMsgId: TalkingMsgId,
            key: fileContent  // Storing key as a string
        };

        const jsonData = JSON.stringify(jsonObj);

        ws.send(jsonData);
    };

    reader.readAsText(file); // Reads file as a string
}

function downloadSensorGroupKey() {
    window.location.href = "/download";
}



//SOS mac address
// Validation for inputs inside MAC_input div


const inputR = document.getElementById('inputR');
const inputG = document.getElementById('inputG');
const inputB = document.getElementById('inputB');
const previewBox = document.getElementById('colorPreview');

function updateColor() {
    const r = parseInt(inputR.value) || 0;
    const g = parseInt(inputG.value) || 0;
    const b = parseInt(inputB.value) || 0;
    previewBox.style.backgroundColor = `rgb(${r}, ${g}, ${b})`;
}

// Add event listeners to inputs
[inputR, inputG, inputB].forEach(input => {
    input.addEventListener('input', updateColor);
});





// Initialize the color picker


function Find() {
    const ids = ['MAC_input1', 'MAC_input2', 'MAC_input3', 'MAC_input4', 'MAC_input5', 'MAC_input6'];
    const macBytes = [];
    const macHexStrings = [];
    let hasError = false;

    ids.forEach((id, index) => {
        const input = document.getElementById(id);
        const val = input.value.trim();

        // Always treat input as hex
        const byteVal = parseInt(val, 16);

        if (isNaN(byteVal) || byteVal < 0 || byteVal > 255) {
            hasError = true;
            console.log(`❌ Input ${index + 1} invalid or out of range: "${val}"`);
            input.style.border = '2px solid red';
        } else {
            macBytes.push(byteVal);
            macHexStrings.push(byteVal.toString(16).padStart(2, '0').toUpperCase());
            input.style.border = '';
        }
    });

    if (hasError) {
        console.log('🚫 MAC data not sent due to input errors.');
        return;
    }

    // Debug: show final MAC address in HEX
    console.log("🔷 MAC Address (Hex):", macHexStrings.join(':'));

    const getColorComponent = (id) => {
        const val = parseInt(document.getElementById(id).value);
        return Math.max(0, Math.min(255, isNaN(val) ? 0 : val));
    };

    const r = getColorComponent("inputR");
    const g = getColorComponent("inputG");
    const b = getColorComponent("inputB");

    const checkboxByte = document.getElementById('enableMAC').checked ? 1 : 0;

    macBytes.push(checkboxByte, r, g, b);
    const header = [2, 0, 2];
    const finalPayload = new Uint8Array([...header, ...macBytes]);

    ws.send(finalPayload);

    sosSend = true;
    console.log("✅ Payload sent:", finalPayload);
}



function ackSOS() {
    const ids = ['MAC_input1', 'MAC_input2', 'MAC_input3', 'MAC_input4', 'MAC_input5', 'MAC_input6'];
    const macBytes = [];
    let hasError = false;
    const macHexStrings = [];
    ids.forEach((id, index) => {
        const input = document.getElementById(id);
        const val = input.value.trim();

        // Always treat input as hex
        const byteVal = parseInt(val, 16);

        if (isNaN(byteVal) || byteVal < 0 || byteVal > 255) {
            hasError = true;
            console.log(`❌ Input ${index + 1} invalid or out of range: "${val}"`);
            input.style.border = '2px solid red';
        } else {
            macBytes.push(byteVal);
            macHexStrings.push(byteVal.toString(16).padStart(2, '0').toUpperCase());
            input.style.border = '';
        }
    });

    if (hasError) {
        console.log('🚫 MAC data not sent due to input errors.');
        return;
    }

    // Checkbox flag
    const checkbox = document.getElementById('enableMAC');
    const checkboxByte = checkbox.checked ? 1 : 0;
    macBytes.push(checkboxByte);

    // Payload build
    const header = [2, 0, 4];
    const finalPayload = new Uint8Array([...header, ...macBytes]);

    ws.send(finalPayload);

    sosSend = true;
    console.log("✅ ackSOS payload sent:", finalPayload);
}


function sendCmd() {
    const ids = ['MAC_input1', 'MAC_input2', 'MAC_input3', 'MAC_input4', 'MAC_input5', 'MAC_input6'];
    const macBytes = [];
    const macHexStrings = [];
    let hasError = false;

    ids.forEach((id, index) => {
        const input = document.getElementById(id);
        const val = input.value.trim();

        // Always treat input as hex
        const byteVal = parseInt(val, 16);

        if (isNaN(byteVal) || byteVal < 0 || byteVal > 255) {
            hasError = true;
            console.log(`❌ Input ${index + 1} invalid or out of range: "${val}"`);
            input.style.border = '2px solid red';
        } else {
            macBytes.push(byteVal);
            macHexStrings.push(byteVal.toString(16).padStart(2, '0').toUpperCase());
            input.style.border = '';
        }
    });

    if (hasError) {
        console.log('🚫 MAC data not sent due to input errors.');
        return;
    }

    // Debug: show final MAC address in HEX
    console.log("🔷 MAC Address (Hex):", macHexStrings.join(':'));

    const getColorComponent = (id) => {
        const val = parseInt(document.getElementById(id).value);
        return Math.max(0, Math.min(255, isNaN(val) ? 0 : val));
    };
    cmdSerialNumber = Math.floor(Math.random() * 256);

    const ch01 = getColorComponent("switch-01");
    const ch02 = getColorComponent("switch-02");
    const ch03 = getColorComponent("switch-03");
    const ch04 = getColorComponent("switch-04");
    const ch05 = getColorComponent("switch-05");


    macBytes.push(cmdSerialNumber, ch01, ch02, ch03, ch04, ch05);
    const header = [2, 0, 5];
    const finalPayload = new Uint8Array([...header, ...macBytes]);

    ws.send(finalPayload);

    sosSend = true;
    console.log("✅ cmd Payload sent:", finalPayload);
}


document.getElementById("del-btn").addEventListener("click", function () {
    let leavingJson = {};
    let TalkingName = document.getElementById("del-btn");//del-btn
    let delMsgName = TalkingName.textContent;
    switch (SessionGroup) {
        case 32:
            if (confirm(delMsgName + "  conversion history will lost forever,are you sure .")) {
                deleteMsg("Whisper", TalkingMsgId);
                showNotification(3000, delMsgName + "  history have been deleted");
            };
            break;
        case 64:
            if (confirm(delMsgName + " chat history will lost forever,are you sure .")) {
                deleteMsg("Meeting", TalkingMsgId);
                showNotification(3000, delMsgName + "  chat history have been deleted");
            };
            break;
        case 4:
            if (confirm(delMsgName + "  history history will lost forever,are you sure .")) {
                deleteMsg("Speech", TalkingMsgId);
                showNotification(3000, delMsgName + "  history have been deleted");
            };

    }
})


document.getElementById("leave-btn").addEventListener("click", function () {
    let leavingJson = {
        T: 2
    };
    let TalkingName = document.getElementById("del-btn");//del-btn
    let leaveName = TalkingName.textContent;
    switch (SessionGroup) {
        case 32:
            if (confirm(leaveName + " won't get in touch with you   ,are you sure .")) {
                leavingJson.sessionGroup = 0;
                leavingJson.sessionIndex = TalkingUser;
                deleteMsg("Whisper", TalkingMsgId);
                ws.send(JSON.stringify(leavingJson));
                showNotification(3000, leaveName + " is not your buddy anymore");

            };
            break;
        case 64:
            if (confirm("leaving the meeting room ${leaveName} ,are you sure .")) {
                leavingJson.sessionGroup = 1;
                leavingJson.sessionIndex = TalkingUser;
                deleteMsg("Meeting", TalkingMsgId);
                ws.send(JSON.stringify(leavingJson));
                showNotification(3000, leaveName + " won't in your   in meeting room  anymore ");
                setTimeout(() => {
                    window.location.reload();
                }, 5000); // Reloads after 1 seconds
            };
            break;

    }
})



document.getElementById("addMeb-btn").addEventListener("click", function () {
    document.getElementById("addMeb-form").style.display = "block";
    renderWhisperList(WhisperList);
})

//---------------------------------------------float-form


// Picture Preview Functionality

const imageInput = document.getElementById('imageInput');
const convertBtn = document.getElementById('convert');
const magicToolBtn = document.getElementById('magicTool');
const resizeOptions = document.getElementById('resizeOptions');
const preview = document.getElementById('preview');
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const downloadLink = document.getElementById('downloadLink');
let img = new Image();

// Load image and display preview
imageInput.addEventListener('change', (event) => {
    const file = event.target.files[0];
    if (file) {
        const reader = new FileReader();
        reader.onload = (e) => {
            img.src = e.target.result;
            img.onload = () => {
                canvas.width = img.width;
                canvas.height = img.height;
                ctx.drawImage(img, 0, 0);
                preview.style.display = 'block';
            };
        };
        reader.readAsDataURL(file);
    }
});

// Resize image while maintaining aspect ratio
resizeOptions.addEventListener('change', () => {
    const scale = parseFloat(resizeOptions.value);
    canvas.width = img.width * scale;
    canvas.height = img.height * scale;
    ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
});

// Apply blur effect when mouse moves
magicToolBtn.addEventListener('mousedown', () => {
    preview.addEventListener('mousemove', blurEffect);
});

preview.addEventListener('mouseup', () => {
    preview.removeEventListener('mousemove', blurEffect);
});

preview.addEventListener('touchmove', (event) => {
    event.preventDefault();
    blurEffect(event); // Apply blur as the finger moves
});


// Convert image to WebP format and create a download link
convertBtn.addEventListener('click', () => {
    canvas.toBlob((blob) => {
        if (blob) {
            const url = URL.createObjectURL(blob);
            WebpBlob = blob;
            downloadLink.href = url;
            downloadLink.textContent = `Download WebP (${(blob.size / 1024).toFixed(2)} KB)`;
            downloadLink.style.display = 'block';
        } else {
            showNotification(3000, "WebP conversion failed. Try again!");
        }
    }, 'image/webp', 0.2);
});

// GPS monitor----------------------------------------------------------------------------------------------------------------






function fetchMonitor(gpsJson) {

    document.getElementById("speed").innerText = `Speed: ${gpsJson.Sp} km/h`;
    document.getElementById("distance").innerText = `Distance: ${gpsJson.Td} meters`;
    document.getElementById("latitude").innerText = `Latitude: ${gpsJson.La}`;
    document.getElementById("longitude").innerText = `Longitude: ${gpsJson.Ln}`;
    document.getElementById("altitude").innerText = `Altitude: ${gpsJson.Al}`;
    document.getElementById("relay").innerText = `RelayNum: ${gpsJson.Ry}`;
    document.getElementById("status").innerText = `Status: ${gpsJson.St}`;
}





// Handle PNG background upload and display
document.getElementById('tileBackground').addEventListener('change', function (event) {

    if (!localStorage.getItem("zipLoaded")) {

        let script = document.createElement("script");
        script.src = "zip.js"; // Replace with your JS file
        document.body.appendChild(script);
        localStorage.setItem("zipLoaded", "true");
    }


    const file = event.target.files[0];
    if (file) {
        const reader = new FileReader();
        reader.onload = function (e) {
            document.getElementById('map').style.backgroundImage = `url(${e.target.result})`;
            document.getElementById('map').style.backgroundSize = "cover"; // Ensure full coverage
        };
        reader.readAsDataURL(file);
    } else {
        document.getElementById('map').style.backgroundColor = '#ddd';  // Default color if no image is uploaded
    }
});









// GPS monitor----------------------------------------------------------------------------------------------------------------




//---------------------------------------------websocket 
window.onload = function () {
    // clear speech DB
    localStorage.clear();
    loadingMsgDB();
    if (sessionStorage.getItem("reloaded")) {
        //
    } else {
        deleteMsg("Speech", 0);

    }





    busyNotifyUpdate();


    // Set a flag in sessionStorage
    sessionStorage.setItem("reloaded", "true");
    ws.binaryType = "arraybuffer";

    ws.onopen = () => {
        if (!sessionStorage.getItem("reloaded"))
            showNotification(3000, "let's talk");


        try {
            if (audioContext.state === 'suspended') audioContext.resume();
            initializeWebSocketAndDecoder();
            alert("audio listen");
            document.getElementById('status').textContent = 'Streaming';
        } catch (e) {
            console.error('Start failed:', e);
            document.getElementById('status').textContent = 'Start failed';
        }

    }

    ws.onerror = (e) => {
        console.error("WebSocket Error:", error);
        showNotification(3000, "WebSocket Error:", error);
    }

    ws.onclose = () => {
        showNotification(3000, "network disconnected");
        if (decoder) {
            decoder.close();
            decoder = null;
        }

        document.getElementById('status').textContent = 'Stopped';
    }
    ws.onmessage = async (event) => {



        if (event.data instanceof Blob) {
            // Convert Blob to Uint8Array 
            const arrayBuffer = event.data.arrayBuffer();
            data = new Uint8Array(arrayBuffer);
            try {
                //const data = event.data instanceof Blob ? new Uint8Array(await event.data.arrayBuffer()) : new Uint8Array(event.data);


                const len1 = data[1];
                const len2 = data[2];
                if (3 + len1 + len2 > data.length) {
                    console.warn('Invalid chunk lengths');
                    return;
                }

                const chunk1 = data.slice(3, 3 + len1);
                const chunk2 = data.slice(3 + len1, 3 + len1 + len2);

                const encodedChunk1 = new EncodedAudioChunk({
                    type: 'key',
                    timestamp: (frameIndex++ * frameDurationMs * 1000),
                    data: chunk1
                });
                const encodedChunk2 = new EncodedAudioChunk({
                    type: 'key',
                    timestamp: (frameIndex++ * frameDurationMs * 1000),
                    data: chunk2
                });

                if (decoder.decodeQueueSize < 8) {
                    await decoder.decode(encodedChunk1);
                    await decoder.decode(encodedChunk2);
                }
            } catch (e) {
                console.error('WebSocket processing error:', e);
            }

        } else if (event.data instanceof ArrayBuffer) {

            //-----------------------------------websocket received binary
            const byteArray = new Uint8Array(event.data);


            const rcvCode = byteArray[0];  // Determines which database to use

            busyNotify++;



            switch (rcvCode) {
                case 1://phone
                    {
                        //receiveChunk(byteArray);
                        try {

                            const len1 = byteArray[1];
                            const len2 = byteArray[2];
                            if (3 + len1 + len2 > byteArray.length) {
                                console.warn('Invalid chunk lengths');
                                return;
                            }

                            const chunk1 = byteArray.slice(3, 3 + len1);
                            const chunk2 = byteArray.slice(3 + len1, 3 + len1 + len2);

                            const encodedChunk1 = new EncodedAudioChunk({
                                type: 'key',
                                timestamp: (frameIndex++ * frameDurationMs * 1000),
                                data: chunk1
                            });
                            const encodedChunk2 = new EncodedAudioChunk({
                                type: 'key',
                                timestamp: (frameIndex++ * frameDurationMs * 1000),
                                data: chunk2
                            });

                            if (decoder.decodeQueueSize < 8) {
                                await decoder.decode(encodedChunk1);
                                await decoder.decode(encodedChunk2);
                            }
                        } catch (e) {
                            console.error('WebSocket processing error:', e);
                        }
                    }
                    break;
                case 2://alert
                    {
                        //MAC address parse

                        let sosOffset = 0;

                        if (byteArray[1] == 0)
                            sosOffset = 0;
                        else
                            sosOffset = 12;

                        const hexString = Array.from(byteArray.slice(3 + sosOffset, 9 + sosOffset))
                            .map(byte => byte.toString(16).padStart(2, '0')) // Convert to 2-digit hex
                            .join(':'); // Join with colon

                        console.log(hexString); // e.g., "1a:2b:3c:4d:5e:6f"

                        if (byteArray[1] < 3) {

                            if (byteArray[2] == 1) {//watchSOS

                                //byteArray.length 
                                const jsonBytes = byteArray.slice(11);
                                const trimmedArray = jsonBytes.slice(0, jsonBytes.length - 1);
                                const decoder = new TextDecoder("utf-8");
                                let jsonStr = decoder.decode(trimmedArray);//.replace(/[^\p{L}\p{N}\s]/gu, ""); 
                                jsonStr = "watchSOS MAC : " + hexString + ".  message : " + jsonStr;
                                searchAlertHistory(jsonStr);
                                if (!foundMatch) {
                                    showNotification(10000, jsonStr);
                                    storeMsg("Speech", 0, 1, jsonStr, true);//#define MSG 2   // text message
                                }

                            }

                            if (byteArray[2] == 0) {//phoneSOS

                                let list1Value = byteArray[9 + sosOffset];
                                let list2Value = byteArray[10 + sosOffset];
                                let list3Value = byteArray[11 + sosOffset];
                                let list4Value = byteArray[12 + sosOffset];


                                let Type = document.getElementById("list1").options[list1Value].text //
                                let Fatality = document.getElementById("list2").options[list2Value].text //
                                let Struck = document.getElementById("list3").options[list3Value].text //
                                let Happened = document.getElementById("list4").options[list4Value].text //
                                let Note = MsgTextDecoder.decode(byteArray.slice(13 + sosOffset));
                                let alertRcvMsgInput = `SOS ${Happened}  ${Type}  happened .  ${Fatality} FATAL  and ${Struck} STUCK ; Note:${Note};`;
                                let alertRcvMsg = alertRcvMsgInput.replace(/[^\p{L}\p{N}\s]/gu, "");
                                alertRcvMsg = "phoneSOS MAC : " + hexString + ".  message : " + alertRcvMsg;
                                searchAlertHistory(alertRcvMsg);
                                if (!foundMatch) {
                                    showNotification(10000, alertRcvMsg);
                                    storeMsg("Speech", 0, 1, alertRcvMsg, true);//#define MSG 2   // text message

                                }


                            }

                        }



                    }
                    break;
                case 4://speech WEB:code,tag,Payload_V
                    {
                        const listIdx = byteArray[1]; // Extracts the session ID                          
                        const speechRcvArray = byteArray.slice(2);
                        storeMsg("Speech", SpeechList[listIdx].msgId, 2, speechRcvArray, true);//#define MSG 2   // text message
                    }
                    break;
                case 32://whisper WEB:code,listIdx,dataType,meta
                    {
                        const listIdx = byteArray[1]; // Extracts the session ID                        
                        const dataType = byteArray[2]; // Extracts the data type
                        const whisperRcvArray = byteArray.slice(3);
                        if (dataType == 2)
                            storeMsg("Whisper", WhisperList[listIdx].msgId, dataType, whisperRcvArray, true);
                        if (dataType == 3) {
                            processReceivedData(byteArray);

                        }
                        if (dataType == 6) {
                            if (byteArray[3] == 1)
                                showNotification(3000, "channel is occupied");
                            if (byteArray[3] == 2)
                                showNotification(3000, "channel is free up");
                        }

                    }
                    break;
                case 64://meeting WEB:code,listIdx,dataType,meta
                    {
                        const listIdx = byteArray[1]; // Extracts the session ID                        
                        const dataType = byteArray[2]; // Extracts the data type
                        const meetingRcvArray = byteArray.slice(3);
                        if (dataType == 2)
                            storeMsg("Meeting", MeetingList[listIdx].msgId, dataType, meetingRcvArray, true);
                        if (dataType == 3) {
                            processReceivedData(byteArray);
                        }
                        if (dataType == 6) {
                            if (byteArray[3] == 1)
                                showNotification(3000, "channel is occupied");
                            if (byteArray[3] == 2)
                                showNotification(3000, "channel is free up");
                        }
                    }
                    break;
                case 128:
                    {
                        if (byteArray[1] == 5)//greet
                            storeMsg("Speech", 0, 42, byteArray.slice(1), true);//#define MSG 2   // text message
                        if (byteArray[1] == 6)//confirm
                            storeMsg("Speech", 0, 43, byteArray.slice(1), true);//#define MSG 2   // text message

                    }
                    break;




            }



        } else if (typeof event.data == "string") {
            //-----------------------------------websocket received string
            // 
            //const rcvjsonData = JSON.parse(event.data);
            const rcvjsonData = typeof event.data === 'string' ? JSON.parse(event.data) : event.data;



            switch (rcvjsonData["T"]) {
                case 2:
                    {
                        if (rcvjsonData.hasOwnProperty("WhisperList")) {
                            WhisperList = WhisperList.concat(rcvjsonData.WhisperList);
                            if (ActiveTab == 32)
                                renderUserList(WhisperList);
                        }
                        if (rcvjsonData.hasOwnProperty("MeetingList")) {
                            //Object.assign(MeetingList, rcvjsonData["MeetingList"]);

                            MeetingList = MeetingList.concat(rcvjsonData.MeetingList);
                            if (ActiveTab == 64)
                                renderUserList(MeetingList);
                        }
                        if (rcvjsonData.hasOwnProperty("SpeechList")) {
                            //Object.assign(SpeechList, rcvjsonData["SpeechList"]);
                            SpeechList = SpeechList.concat(rcvjsonData.SpeechList);
                            if (ActiveTab == 4)
                                renderUserList(SpeechList);
                        }
                        //showNotification(3000,event.data);
                    }
                    break;
                case 1:
                    {
                        Object.assign(ConfigList, rcvjsonData);
                        MyName = rcvjsonData["MyName"];

                    }
                    break;
                case 0:
                    showNotification(3000, rcvjsonData["msg"]);
                    break;
                case 3: {
                    showNotification(3000, "page would reload in 5 seconds,save your input as soon as possible");
                    setTimeout(() => {
                        window.location.reload();
                    }, 5000); // Reloads after 1 seconds
                } break;
                case 128:
                    {
                        //
                    }
                    break;
                case 5:
                    //update gps
                    {
                        if (rcvjsonData["W"] == 0)
                            gpsJsonStr = JSON.stringify(rcvjsonData);

                        const timestamp = new Date().toISOString();

                        sessionStorage.setItem(timestamp, JSON.stringify(rcvjsonData));
                        fetchMonitor(rcvjsonData);

                    }
                    break;
            }

        }

    }


}