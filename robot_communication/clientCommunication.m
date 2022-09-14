classdef clientCommunication
   properties
       socket = ""
       message_size = ""
       data = 0;
   end
   methods
       function obj = clientCommunication(ip, port, message_size)
           obj.socket = tcpclient(ip, port);
           obj.message_size = message_size;
       end
       function send_data(obj, mssg)
           if obj.socket ~= ""
               write(obj.socket, mssg, "string");
           end
       end
       function [mssg, obj] = receive_data(obj)
           mssg = read(obj.socket, obj.message_size, "string");
           mssg = str2num(mssg);
           obj.data = mssg;
       end
       function obj = close(obj)
           obj.socket.close()
       end
       
   end
    
end