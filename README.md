# Smart Helmet – Accident Detection & Alert System (Python)

## 📌 Overview
The **Smart Helmet** project is designed to enhance rider safety by detecting accidents (falls or crashes) using an accelerometer and automatically sending an alert message with the rider’s location.  
This Python-based implementation simulates sensor inputs and alert mechanisms, making it easy to demonstrate even without hardware.

---

## 🚀 Features
- **Fall / Crash Detection** based on acceleration threshold  
- **GPS Location Fetching** (simulated)  
- **Emergency SMS Alert System** (simulated)  
- **LED & Buzzer Alert** (simulated)  
- **Well-structured Python code** with modules  
- Suitable for **final-year projects, interviews, Cognizant submission**

---

## 📁 Project Structure

smart_helmet/
│
├── main.py # Main control logic
├── sensors.py # Accelerometer & GPS sensor simulation
├── alert.py # SMS alert module
├── hardware.py # LED/Buzzer control (simulated)
└── config.py # Configuration values


---

## 🛠 Technologies Used
- **Python 3**
- Object-Oriented Programming (OOP)
- Modular file-based architecture  
- Simulated hardware environment

---

## ⚙ How It Works
1. **Accelerometer (simulated)** continuously reads acceleration values.  
2. If the total acceleration exceeds the threshold → **fall detected**.  
3. System fetches **GPS location**.  
4. **SMS alert** is triggered with location.  
5. Helmet **LED and buzzer activate** to notify nearby people.

---

## ▶ Running the Project

### **1. Navigate to the project folder**
cd smart_helmet

### **2. Run the main file**
python main.py

### **3. You will see simulated output like:**
⚠ FALL DETECTED!
Accident detected! Location: 17.445° N, 78.349° E
🔴 LED ON
🔊 BUZZER ON
--- SMS ALERT TRIGGERED ---
To: +911234567890
Message: Accident detected!

---

## 🧪 Simulation Details
- The accelerometer generates small random values (normal riding).  
- Every 15 seconds, a spike is generated to simulate a crash.  
- GPS gives a fixed sample location (you can modify it).

---

## 📝 Configuration
All adjustable values are stored in **config.py**:

FALL_THRESHOLD = 3.0
ALERT_PHONE = "+911234567890"

You can change:
- Fall sensitivity  
- Emergency phone number  
- Location format  

---

## 📜 Author
**Rishith**  
Smart Helmet – Python Version  
(For Cognizant Project Submission)

---

## ✔ Final Notes
- This project can run on **any computer** (no hardware needed).  
- Clean, modular structure is suitable for companies reviewing your code.  
- You can extend this for real hardware later (Raspberry Pi + sensors).

---

If you want, I can also prepare:
📄 **Project Report PDF**  
📁 **ZIP file of full project**  
📊 **Block diagram**  
🧪 **Output screenshots**

Just tell me!

