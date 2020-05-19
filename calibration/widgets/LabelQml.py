from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5 import QtQuickWidgets 

class LabelQml(QtQuickWidgets.QQuickWidget):
        
    def __init__(self, parent=None):
        super(LabelQml, self).__init__(parent)
        self.setSource(QtCore.QUrl.fromLocalFile("widgets/label.qml"))
        self.setResizeMode(QtQuickWidgets.QQuickWidget.SizeRootObjectToView)
        self.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.setFixedHeight(100)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop)
        self.setClearColor(QtCore.Qt.transparent)

    def text(self):
        return self.rootObject().property("text") if self.rootObject() else  ""

    def setText(self, text):
        if self.rootObject() and self.text() != text:
           self.rootObject().setProperty("text", text)

    def color(self):
        return self.rootObject().property("color") if self.rootObject() else  ""

    def setColor(self, color):
        color = QtGui.QColor(color)
        if self.rootObject() and self.color() != color:
            self.rootObject().setProperty("color", color)

    def font(self):
        return self.rootObject().property("font") if self.rootObject() else  ""

    def setFont(self, font):
        if self.rootObject() and self.font() != font:
            self.rootObject().setProperty("font", font)
            