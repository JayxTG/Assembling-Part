import sys
import time
import csv
import serial
import serial.tools.list_ports
import threading
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QPushButton,
    QLineEdit, QFormLayout, QFileDialog, QDialog, QMessageBox, QComboBox
)

class Communicator(QObject):
    data_received = pyqtSignal(str)
    finished = pyqtSignal()
    error_occurred = pyqtSignal(str)

class SensorDataLoggerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Sensor Data Logger")
        self.setGeometry(100, 100, 400, 300)
        self.setWindowIcon(QIcon('icon.jpg'))

        self.layout = QVBoxLayout()
        self.icon_label = QLabel(self)
        self.icon_label.setPixmap(QIcon('icon.jpg').pixmap(100, 100))
        self.icon_label.setAlignment(Qt.AlignCenter)

        self.name_label = QLabel('Sensor Data Logger', self)
        self.name_label.setFont(QFont('Arial', 20))
        self.name_label.setAlignment(Qt.AlignCenter)

        self.layout.addWidget(self.icon_label)
        self.layout.addWidget(self.name_label)

        QTimer.singleShot(2000, self.fadeOut)
        self.setLayout(self.layout)

    def fadeOut(self):
        self.icon_label.hide()
        self.name_label.hide()
        self.showComPorts()

    def showComPorts(self):
        self.ports = list(serial.tools.list_ports.comports())
        self.com_label = QLabel('Select a COM Port:', self)
        self.layout.addWidget(self.com_label)

        self.com_buttons_layout = QVBoxLayout()
        for port in self.ports:
            btn = QPushButton(port.device, self)
            btn.clicked.connect(lambda checked, p=port.device: self.selectPort(p))
            self.com_buttons_layout.addWidget(btn)

        self.layout.addLayout(self.com_buttons_layout)

    def selectPort(self, port):
        self.port = port
        self.clearLayout(self.com_buttons_layout)
        self.com_label.hide()
        self.showBaudRateSelection()

    def showBaudRateSelection(self):
        self.baud_rate_label = QLabel('Select Baud Rate:', self)
        self.baud_rate_combo = QComboBox(self)
        self.baud_rate_combo.addItems(['9600', '19200', '38400', '57600', '115200'])
        self.layout.addWidget(self.baud_rate_label)
        self.layout.addWidget(self.baud_rate_combo)

        self.next_button = QPushButton('Next', self)
        self.next_button.clicked.connect(self.startSensorDataLogging)
        self.layout.addWidget(self.next_button)

    def startSensorDataLogging(self):
        try:
            self.baud_rate = int(self.baud_rate_combo.currentText())
            self.clearLayout(self.layout)

            self.csv_file_path = self.openFileDialog()
            if self.csv_file_path:
                self.runSensorDataLogging()
        except ValueError:
            self.showError('Invalid baud rate input')
            return

    def openFileDialog(self):
        save_dialog = QFileDialog(self, 'Save CSV File')
        save_dialog.setAcceptMode(QFileDialog.AcceptSave)
        save_dialog.setDefaultSuffix('csv')
        if save_dialog.exec_() == QDialog.Accepted:
            return save_dialog.selectedFiles()[0]
        return None

    def runSensorDataLogging(self):
        try:
            self.serial_connection = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.csv_file = open(self.csv_file_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Timestamp', 'Sensor Value'])

            self.stop_event = threading.Event()
            self.communicator = Communicator()
            self.communicator.data_received.connect(self.saveSensorData)
            self.communicator.finished.connect(self.showCompletionOptions)
            self.communicator.error_occurred.connect(self.handleError)

            self.thread = threading.Thread(target=self.readSensorData)
            self.thread.start()
        except Exception as e:
            self.handleError(str(e))

    def readSensorData(self):
        try:
            while not self.stop_event.is_set():
                if self.serial_connection.in_waiting:
                    line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    self.communicator.data_received.emit(line)
            self.serial_connection.close()
            self.csv_file.close()
            self.communicator.finished.emit()
        except Exception as e:
            self.communicator.error_occurred.emit(str(e))

    def saveSensorData(self, line):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        sensor_value = line.strip()
        self.csv_writer.writerow([timestamp, sensor_value])

    def showCompletionOptions(self):
        self.clearLayout(self.layout)
        message = QLabel('Sensor data logging completed. What would you like to do?', self)
        self.layout.addWidget(message)

        record_again_button = QPushButton('Record again', self)
        record_again_button.clicked.connect(self.resetApp)
        finish_button = QPushButton('Finish', self)
        finish_button.clicked.connect(self.close)

        self.layout.addWidget(record_again_button)
        self.layout.addWidget(finish_button)

    def resetApp(self):
        self.stop_event.set()
        self.clearLayout(self.layout)
        self.showComPorts()

    def clearLayout(self, layout):
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def handleError(self, message):
        self.stop_event.set()
        self.clearLayout(self.layout)
        self.showCompletionOptions()
        self.showError(f'An error occurred: {message}')

    def showError(self, message):
        error_dialog = QDialog(self)
        error_dialog.setWindowTitle('Error')
        error_layout = QVBoxLayout()
        error_label = QLabel(message, error_dialog)
        error_layout.addWidget(error_label)
        ok_button = QPushButton('OK', error_dialog)
        ok_button.clicked.connect(error_dialog.accept)
        error_layout.addWidget(ok_button)
        error_dialog.setLayout(error_layout)
        error_dialog.exec_()

    def closeEvent(self, event):
        self.stop_event.set()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = SensorDataLoggerApp()
    ex.show()
    sys.exit(app.exec_())
