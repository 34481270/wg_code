# coding:UTF-8
from WG.sensor.i_data_processor import IDataProcessor # type: ignore

"""
    JY901S数据处理器
"""


class JY901SDataProcessor(IDataProcessor):
    onVarChanged = []
    def onOpen(self, deviceModel):
        pass

    def onClose(self):
        pass

    @staticmethod
    def onUpdate(*args):
        for fun in JY901SDataProcessor.onVarChanged:
            fun(*args)