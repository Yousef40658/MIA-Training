import zlib #CRC  


class codec:
    #---------
    # encoding
    #---------
    def encode(self, list_of_commands):
        # Checksum -added by CRC- to ensure the data was sent correctly
        if not list_of_commands :
            #print ("empty") 
            return " "
        raw_data = ""
        for command in list_of_commands:
            raw_data += str(len(command)) + "#" + command  # length prefix -- 
        #https://www.geeksforgeeks.org/python/zlib-crc32-in-python/
        checksum = zlib.crc32(raw_data.encode())                    # returns  unsigned 32 bit 
        encoded = str(checksum) + "#" + raw_data                  
        return encoded

    #----------
    # decoding
    #----------
    def decode(self, encoded):
        decoded = []
        if encoded.isspace() :
            #print("Empty while decoding")
            return []
        i = encoded.find("#")                                                         #returns index of '#' and -1 in failure 
        if (i == -1) :
            print("Missing checksum separator")
            return []
        try :
            expected_checksum = int(encoded[:i])                                    #in try: in case any char turned into integer before comparison
        except ValueError:
            print("Invalid checksum format")
            return []
        
        raw_data = encoded[i+1:]                                                                      # rest of the encoded string
        actual_checksum = zlib.crc32(raw_data.encode())
        if expected_checksum != actual_checksum:
            print("Checksum mismatch! Data may be corrupted.")
            return []
        #decoding the string 
        i = 0
        while i < len(raw_data):                     
            j = i
            while j < len(raw_data) and raw_data[j] != "#":                                   #reaching separetor 
                j += 1
            if j == len(raw_data):
                break  
            try:
                length = int(raw_data[i:j])  
            except ValueError:
                break
            j += 1
            decoded.append(raw_data[j:j+length])  
            i = j + length
        return decoded

# Create instance
code = codec()

#-----------------------
encoded = code.encode(["Push", "Box,box", "Push", "Overtake" , " ", "", "#!@#!s"])
if  encoded :                                                                                                         #to handle empty list output 
    print("Encoded string:")
else :
    print ("__no_input__")

#----------------------

#----------------------
decoded = code.decode(encoded)
print(decoded)
#-----------------------
