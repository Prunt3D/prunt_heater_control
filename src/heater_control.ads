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

with Physical_Types;          use Physical_Types;
with Ada.Real_Time;
with Ada.Task_Identification; use Ada.Task_Identification;

generic
   type Heater_Name is (<>);
   with procedure Set_Heater_PWM (Heater : Heater_Name; PWM : PWM_Scale);
   type Thermistor_Name is (<>);
   with function Get_Thermistor_Voltage (Thermistor : Thermistor_Name) return Voltage;
   type User_Data is private;
   with procedure Emergency_Stop;
package Heater_Control is

   procedure Check_In;

   type Voltage_To_Temperature_Type is access function (V : Voltage; Data : User_Data) return Temperature;

   type Control_Kind is (PID_Kind, Bang_Bang_Kind, Always_Off_Kind);

   type Control_Parameters (Kind : Control_Kind := Always_Off_Kind) is record
      case Kind is
         when PID_Kind =>
            Kp : Inverse_Temperature;
            Ki : Frequency_Over_Temperature;
            Kd : Time_Over_Temperature;
         when Bang_Bang_Kind =>
            Max_Delta : Temperature;
         when Always_Off_Kind =>
            null;
      end case;
   end record;

   task type Control_Loop is
      entry General_Setup
        (In_Heater                 : Heater_Name;
         In_Thermistor             : Thermistor_Name;
         In_VTT_Data               : User_Data;
         In_Voltage_To_Temperature : Voltage_To_Temperature_Type;
         In_Minimum_Temperature    : Temperature;
         In_Maximum_Temperature    : Temperature;
         In_Control_Params         : Control_Parameters);
      entry Run (In_Target : Temperature);
      entry Stop;
      entry Wait_For_Temperature;
      entry Wait_For_Stop;
   end Control_Loop;

   Control_Loops : array (Heater_Name) of Control_Loop;

private

   subtype Temperature_Time_Integral is Dimensioned_Float with
     Dimension => (Symbol => "°Cs", Second => 1, Celcius => 1, others => 0);

   Loop_Period             : constant Time := 0.1 * s;
   Loop_Count_Grace_Period : constant      := 5;

   Loop_Period_Time_Span : constant Ada.Real_Time.Time_Span := Ada.Real_Time.Milliseconds (Integer (Loop_Period / ms));

   type Heater_Owner_Type is array (Heater_Name) of Task_Id;
   type Is_Running_Type is array (Heater_Name) of Boolean;
   type Last_Refresh_Type is array (Heater_Name) of Ada.Real_Time.Time;

   protected Watchdog is
      procedure Check_In;
      procedure Start (Heater : Heater_Name);
      procedure Stop (Heater : Heater_Name);
      procedure Refresh (Heater : Heater_Name);
   private
      procedure Check_Owner (Heater : Heater_Name; Id : Task_Id);

      Heater_Owner : Heater_Owner_Type := [others => Null_Task_Id];
      Is_Running   : Is_Running_Type   := [others => False];
      Last_Refresh : Last_Refresh_Type := [others => Ada.Real_Time.Time_First];
   end Watchdog;

end Heater_Control;
