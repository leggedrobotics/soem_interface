import xml.etree.ElementTree as ET

tree = ET.parse("Beckhoff AX5125-xxxx-0213 Dict.xml")
root = tree.getroot()
root = root[0]

datatypes = root[0]
objects = root[1]

for obj in objects:
    required = False
    props = obj.find("Properties")
    if props:
        for prop in props:
            for elem in prop:
                #print(" ",elem.tag, elem.text)
                if elem.text and  elem.text in "StartupList":
                    required = True
    if not required:
        continue

    index = obj.find("Index")
    
    print(index.tag,  index.text)
    
    idn1 = (int(index.text) & 0x00FF00)>>8
    idn2 = int(index.text) & 0xFF
    
    print(str(idn1)+"-"+str(idn2))
    
    name = obj.find("Name")
    print(name.text)
    
    bitsize = obj.find("BitSize")
    print("Bit size: ", bitsize.text)
    
    info = obj.find("Info")
    if info:
        min_val = ""
        max_val = ""
        def_val = ""
        if info.find("MinValue"):
            min_val = info.find("MinValue").text
        if info.find("MaxValue"):
            max_val = info.find("MaxValue").text
        if info.find("DefaultValue"):
            def_val = info.find("DefaultValue").text
        
        print("Min,Max,Default")
        print(str(min_val),max_val,def_val)
    

         
                   
    
    if required:
        print("!Required for startup!")

    print("")
    
    
    
    
    
    
'''   
     <Object>
        <Index>16467</Index>
        <!-- S-4-0083 -->
        <Name LcId="1033"><![CDATA[Negative torque/force limit value]]></Name>
        <Comment LcId="1033"><![CDATA[The negative torque/force limit value limits the commanded torque/force in the negative direction of rotation.]]></Comment>
        <Type>UINT</Type>
        <BitSize>16</BitSize>
        <Info>
          <MinValue>0</MinValue>
          <MaxValue>1000</MaxValue>
          <DefaultValue>1000</DefaultValue>
        </Info>
        <Flags>
          <Attribute>#x01110001</Attribute>
        </Flags>
        <Properties>
          <Property>
            <Name>UnitName</Name>
            <Desc LcId="1033"><![CDATA[%]]></Desc>
          </Property>
          <Property>
            <Name>PROP_GROUP</Name>
            <Value>4</Value>
          </Property>
          <Property>
            <Name>StartupList</Name>
            <Value>PS</Value>
          </Property>
        </Properties>
      </Object>
      '''
