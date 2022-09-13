classdef ArDrone < handle
    
    properties
        
        % Properties or Parameters
        pCAD   % ArDrone 3D image
        pPar   % Parameters Dynamic Model
        pID    % Identification
        
        % Control variables
        pPos   % Posture
        pSC    % Signals
        pFlag  % Flags
        
        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication                  
        
    end
    methods
        function drone = ArDrone(ID)
            if nargin < 1
                ID = 1;
            end
            
            drone.pID = ID; 
            
            drone.pFlag.Connected = 0;
            
            iParameters(drone);
            iControlVariables(drone);
            
            mCADload(drone);
        end
        
        % ==================================================
        % Initialization parameters
        iControlVariables(drone);
        iParameters(drone);
        
        % ==================================================
        % Drone functions
        % Communication
        rConnect(drone);
        rDisconnect(drone);
        
        % Data request
        rGetStatusRawData(drone);
        rGetStatusRawDataFull(drone);
        rGetSensorData(drone);
        rGetSensorCalibration(drone);
        
        % Takeoff/Landing
        rTakeOff(drone);
        rLand(drone);
        
        % Emergency
        rEmergency(drone)
        
        % Command
        rCommand(drone);
        rSetLed(drone,id,freq,duration);
        rSendControlSignals(drone);
        
        % ==================================================
        % ArDrone 3D Image
        mCADload(drone);
        mCADcolor(drone,cor);
        mCADplot(drone,visible);
        mCADdel(drone);
                        
        % ==================================================
        % ArDrone Models for simulation
        sDynamicModel(drone);               
    end
end