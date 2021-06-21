import xml.etree.ElementTree as ET

class ObjectBrowserYolo:
    def __init__(self, document_name):
        self.__xmltree = ET.parse(document_name)
        self.__documentroot = self.__xmltree.getroot()
    
    def getCategory(self, item_name):
        for item in self.__documentroot.iter('item'):
            name = item.find('name').text
            if name == item_name:
                return item.find('category').text
    
        return 'Unknown'