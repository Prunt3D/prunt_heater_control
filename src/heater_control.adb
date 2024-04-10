-----------------------------------------------------------------------------
--                                                                         --
--                   Part of the Prunt Motion Controller                   --
--                                                                         --
--            Copyright (C) 2024 Liam Powell (liam@prunt3d.com)            --
--                                                                         --
--  This program is free software: you can redistribute it and/or modify   --
--  it under the terms of the GNU General Public License as published by   --
--  the Free Software Foundation, either version 3 of the License, or      --
--  (at your option) any later version.                                    --
--                                                                         --
--  This program is distributed in the hope that it will be useful,        --
--  but WITHOUT ANY WARRANTY; without even the implied warranty of         --
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          --
--  GNU General Public License for more details.                           --
--                                                                         --
--  You should have received a copy of the GNU General Public License      --
--  along with this program.  If not, see <http://www.gnu.org/licenses/>.  --
--                                                                         --
-----------------------------------------------------------------------------

package body Heater_Control is

   use type Ada.Real_Time.Time;
   use type Ada.Real_Time.Time_Span;

   procedure Check_In is
   begin
      Watchdog.Check_In;
   end Check_In;

   task body Control_Loop is
      Heater                 : Heater_Name;
      Thermistor             : Thermistor_Name;
      VTT_Data               : User_Data;
      Voltage_To_Temperature : Voltage_To_Temperature_Type;
      Minimum_Temperature    : Temperature;
      Maximum_Temperature    : Temperature;
      Control_Params         : Control_Parameters;

      Target         : Temperature;
      Is_Running     : Boolean := False;
      Next_Loop_Time : Ada.Real_Time.Time;

      Last_Temperature : Temperature;

      PID_Last_Error : Temperature               := 0.0 * celcius;
      PID_Integral   : Temperature_Time_Integral := 0.0 * celcius * s;
   begin
      accept General_Setup
        (In_Heater                 : Heater_Name;
         In_Thermistor             : Thermistor_Name;
         In_VTT_Data               : User_Data;
         In_Voltage_To_Temperature : Voltage_To_Temperature_Type;
         In_Minimum_Temperature    : Temperature;
         In_Maximum_Temperature    : Temperature;
         In_Control_Params         : Control_Parameters)
      do
         Heater                 := In_Heater;
         Thermistor             := In_Thermistor;
         VTT_Data               := In_VTT_Data;
         Voltage_To_Temperature := In_Voltage_To_Temperature;
         Minimum_Temperature    := In_Minimum_Temperature;
         Maximum_Temperature    := In_Maximum_Temperature;
         Control_Params         := In_Control_Params;
      end General_Setup;

      loop
         Last_Temperature := Voltage_To_Temperature (Get_Thermistor_Voltage (Thermistor), VTT_Data);

         if Last_Temperature < Minimum_Temperature or Last_Temperature > Maximum_Temperature then
            Emergency_Stop;
            Set_Heater_PWM (Heater, 0.0);
         end if;

         select
            accept Run (In_Target : Temperature) do
               if In_Target < Minimum_Temperature or In_Target > Maximum_Temperature then
                  raise Constraint_Error with "Temperature out of range.";  --  TODO: How should this be handled?
               end if;
               Target := In_Target;
            end Run;

            if not Is_Running then
               Is_Running     := True;
               Next_Loop_Time := Ada.Real_Time.Clock;
            end if;
         or
            accept Stop do
               Is_Running := False;
               Set_Heater_PWM (Heater, 0.0);
            end Stop;
         or when Is_Running and then abs (Last_Temperature - Target) / (Target - Minimum_Temperature) < 0.01 =>
            accept Wait_For_Temperature;
         or when Is_Running =>
            accept Wait_For_Stop;
         or when Is_Running =>
            delay until Next_Loop_Time;
            Last_Temperature := Voltage_To_Temperature (Get_Thermistor_Voltage (Thermistor), VTT_Data);
            Next_Loop_Time   := @ + Loop_Period_Time_Span;
            case Control_Params.Kind is
               when PID_Kind =>
                  declare
                     Error  : constant Temperature := Target - Last_Temperature;
                     Output : Dimensionless;
                  begin
                     PID_Integral   := PID_Integral + Error * Loop_Period;
                     Output         :=
                       Control_Params.Kp * Error + Control_Params.Ki * PID_Integral +
                       Control_Params.Kd * (Error - PID_Last_Error) / Loop_Period;
                     PID_Last_Error := Error;
                     Set_Heater_PWM
                       (Heater, Dimensionless'Min (PWM_Scale'Last, Dimensionless'Max (PWM_Scale'First, Output)));
                  end;
               when Bang_Bang_Kind =>
                  if Last_Temperature < Target - Control_Params.Max_Delta then
                     Set_Heater_PWM (Heater, 1.0);
                  end if;

                  if Last_Temperature > Target + Control_Params.Max_Delta then
                     Set_Heater_PWM (Heater, 0.0);
                  end if;
               when Always_Off_Kind =>
                  null;
            end case;
            Watchdog.Refresh (Heater);
         end select;
      end loop;
   end Control_Loop;

   protected body Watchdog is
      procedure Check_In is
         Current_Time : constant Ada.Real_Time.Time := Ada.Real_Time.Clock;
      begin
         for I in Heater_Name loop
            if Is_Running (I) then
               if Last_Refresh (I) > Current_Time or
                 Last_Refresh (I) + Loop_Period_Time_Span * Loop_Count_Grace_Period < Current_Time
               then
                  Emergency_Stop;
                  Set_Heater_PWM (I, 0.0);
               end if;
            end if;
         end loop;
      end Check_In;

      procedure Start (Heater : Heater_Name) is
      begin
         Check_Owner (Heater, Current_Task);
         Is_Running (Heater) := True;
      end Start;

      procedure Stop (Heater : Heater_Name) is
      begin
         Check_Owner (Heater, Current_Task);
         Is_Running (Heater) := False;
      end Stop;

      procedure Refresh (Heater : Heater_Name) is
      begin
         Check_Owner (Heater, Current_Task);
         Last_Refresh (Heater) := Ada.Real_Time.Clock;
      end Refresh;

      procedure Check_Owner (Heater : Heater_Name; Id : Task_Id) is
      begin
         if Heater_Owner (Heater) = Null_Task_Id then
            for I in Heater_Name loop
               if Heater_Owner (I) = Id then
                  Emergency_Stop;
                  Set_Heater_PWM (Heater, 0.0);
                  Set_Heater_PWM (I, 0.0);
               end if;
            end loop;
            Heater_Owner (Heater) := Id;
         end if;

         if Heater_Owner (Heater) /= Id then
            Emergency_Stop;
            Set_Heater_PWM (Heater, 0.0);
         end if;
      end Check_Owner;
   end Watchdog;

end Heater_Control;
