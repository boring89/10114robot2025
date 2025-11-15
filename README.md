# 專案說明

此為FRC10114於2025賽季競賽機器之程式

# 程式碼說明

本專案的主要程式碼皆位於：

src/main/java/robot  
├── Robot.java  
├── RobotContainer.java  
├── commands/  
└── subsystems/  

---

## 目錄內容

### **Robot 主程式 — `Robot.java`**
負責整個機器人的生命週期（初始化、循環執行、模式切換等）。

---

### **RobotContainer — 程式架構中樞**
- 建立並管理所有 **Subsystem（子系統）**
- 設定 **Commands（指令）** 與控制器按鍵
- 提供自動模式（Autonomous）指令
- 管理整個程式的依賴與邏輯關係

---

### **Subsystems — 子系統（`subsystems/`）**
包含機器人的各個功能模組，例如：
- 驅動底盤
- 感測器整合
- 機構控制（Shooter / Intake / Arm 等）

---

### **Commands — 指令（`commands/`）**
定義機器人可執行的動作邏輯，例如：
- 自動對準
- 行駛路徑
- 機構操作


