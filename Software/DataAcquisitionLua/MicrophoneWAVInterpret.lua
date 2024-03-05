local io = require("io")
function GetSerialPortPath()
    -- Run lsusb and capture the output
    local lsusbCommand = "lsusb | grep Teensy"
    local lsusbOutput = io.popen(lsusbCommand):read("*a")
    --Bus 003 Device 004: ID 16c0:0483 Van Ooijen Technische Informatica Teensyduino Serial
    -- Find the line containing the desired device information
    local deviceLine = lsusbOutput:match("Bus %d+ Device %d+: ID " .. "16c0:0483")
    if deviceLine then
        -- Extract the symbolic link name
        local lsserialLine = io.popen("ls /dev/serial/by-id/ | grep Teensy"):read("a")
        local symbolicLink = assert(lsserialLine:match("Teensyduino") ~= nil,
            "failed to find serial Teensy in /dev/serial/by-id/")
        print(lsusbOutput)

        -- Run readlink and capture the output
        local readlinkCommand = "readlink -f /dev/serial/by-id/" .. lsserialLine
        local readlinkOutput = io.popen(readlinkCommand):read("*l")
        print(readlinkOutput)
        return readlinkOutput
    else
        return nil -- Device not found
    end
end

local Serial = require("periphery").Serial
local Clock = require("clock")
local arg1 = tonumber(arg[1]) or 4-- local arg1 = 0
for i = 0, 10 ^ 8, 1 do --stupid wait
    local j = nil
end

local serialPort = assert(GetSerialPortPath(), "no Teensy connected to serial port")
local serial = Serial(serialPort, 9600 * 12 * 5) --requires 57600 Baud Rate Output from micro
local file = io.open("raw.csv", "w")
assert(file ~= nil, "the file couldn't be opened")

file:write("rawValue,filteredValue,normalizedValue,movingAverageResult\n")
-- while (true) do

    local buf = nil
    buf = assert(serial:read(5000 * 4, 5000), "Serial Port Timeout") --reads 5000 4 byte samples, with 5000ms timeout
    if (#buf == 0) then
        print("serial port timed out")
        return
    end
    for i = 1, #buf, 4 * arg1 do
        local rawData = {}
        for j = 0, arg1 - 1 do
            local value = string.unpack("I4", buf, i + 4 * j) -- Assuming the data is unsigned 32-bit integer (use "i4" for signed)
            table.insert(rawData, value)
        end
        local csvLine = table.concat(rawData, ",") .. "\n"
        file:write(csvLine)
    end
    
    file:close()
    serial:close()

-- end

-- if (arg1 == '10') then
--     local file = io.open("raw.wav", "r")

--     local charTable = {} -- Create an empty table to store the characters

--     -- Read the file character by character
--     for char in file:read("*a"):gmatch("..") do --gmatch(".") goes char by char
--         print(char)
--         local numericValue = string.byte(char)  -- Get the numeric value of the character
--         table.insert(charTable, numericValue)   -- Add the numeric value to the table
--     end

--     file:close() -- Close the file

--     -- Print the resulting table
--     for i, value in ipairs(charTable) do
--         local temp1 = ((value & 0xFF00) >> 8)
--         local temp2 = ((value & 0xFF) << 8)
--         print(string.format("%04x", value) ..
--             " 1 " .. string.format("%02x", temp1) .. " and " .. string.format("%02x", temp2))
--         print((((value >> 8) & 0xFF) + ((value << 8) & 0xFF00)) * 3.3 / 1024)
--     end

