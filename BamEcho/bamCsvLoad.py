#==================================================================================================
# BamCsvLoad
# A reuseable module for generic CSV loading.
#
# Takes in valid CSV files, parses them into a dictionary and spits it back out.  There is an
# argument for clarifying CSV's with or without a header row at the beginning. 
#
# Dev Note: I may try to turn this into a larger Utils module for BamEcho or other tools down the 
#           line that could be developed.  I'm just not sure.  It's an arch design decision, and I
#           I don't know if the effort is worth it right now, because I'm not sure what the demand
#           will be in this area.     -DRH 202504
#==================================================================================================


#===============================================================================================
# Bring in the BAM Pos data
#===============================================================================================
def parseCSV(filename, hasHeader):

    # open file
    inFile = open(filename, 'r');
    line = inFile.readline();

    # Priming Read Logic
    line = line.replace("\n","")
    tokens = line.split(",")

    if hasHeader:
        # Detect columns, get the first line and count up the tokens
        csvDataDict = {key: [] for key in tokens};  # dict construction to ensure keys aren't point to same spot in memory
    else: 
        # autogen the column name
        keys = [];
        indx = 0;
        for col in tokens:
            keys.append(f"col_{indx}");
            indx+=1;
     
        csvDataDict = {key: [] for key in keys};  # dict construction to ensure keys aren't point to same spot in memory

        # store the first line in the dictionary since it was data and not a header
        indx = 0;
        for key in csvDataDict.keys():
            if tokens[indx].isnumeric():
                csvDataDict[key] = tokens[indx];
            else:
                csvDataDict[key] = float(tokens[indx]);
            indx+=1;
            
    line = inFile.readline();


    lines = 0;
    # iterate till we have end of the loop
    while line != "":
        line = line.replace("\n","")
        tokens = line.split(",")
        if len(tokens) > 0:

            # store the first line in the dictionary since it was data and not a header
            indx = 0;
            for key, val in csvDataDict.items():
                valueStr = tokens[indx];

                try:
                    numVal = float(valueStr);
                    csvDataDict[key].append(numVal);
                except ValueError:
                    csvDataDict[key].append(valueStr);
                indx+=1;

        # keep reading
        line = inFile.readline();
    #end WHILE
  
    inFile.close()

    return csvDataDict;

#===============================================================================================
#===============================================================================================
def isnum(strValue):
    try:
        numVal = float(strValue);
        return True;
    except ValueError:
        return False;