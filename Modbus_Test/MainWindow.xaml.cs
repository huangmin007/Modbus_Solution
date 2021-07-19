using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Modbus.Device;
using Modbus.Message;
using Modbus.IO;

namespace Modbus_Test
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        private SerialPort serialPort;
        private IModbusMaster master;

        private bool Running = true;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            serialPort = new SerialPort("COM14", 9600);
            serialPort.Open();
            master = ModbusSerialMaster.CreateRtu(serialPort);

#if false
            serialPort.DataReceived += SerialPort_DataReceived;
#else
            ThreadPool.QueueUserWorkItem(new WaitCallback(ThreadFunction), this);
#endif
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            byte[] buffer = new byte[serialPort.BytesToRead];
            int  len = serialPort.Read(buffer, 0, serialPort.BytesToRead);
            
            Console.Write("{0}>>", len);
            for(var i = 0; i < buffer.Length; i ++)
                Console.Write("{0:X2} ", buffer[i]);
            Console.WriteLine();

            byte[] data = new byte[] { 0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x3D, 0xCC };

            //IModbusMessage response = ModbusMessageFactory.CreateModbusRequest(buffer);
            ReadCoilsInputsResponse response = ModbusMessageFactory.CreateModbusMessage<ReadCoilsInputsResponse>(data);
            Console.WriteLine("{0},{1}  {2}", response.FunctionCode, response.SlaveAddress, response.Data);
            //bool[] status = response.Data.Take(8).ToArray<bool>();
            //Console.WriteLine(response);

            Console.WriteLine();
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            Running = false;
            serialPort.Dispose();
            master.Dispose();

            base.OnClosing(e);
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            Button button = (Button)sender;
            if(button == Button_0)
            {
                master.WriteMultipleCoils(1, 0x0000, new bool[8] {false, false, false, false, false, false, false, false });
            }
            else if(button == Button_1)
            {
                master.WriteMultipleCoils(1, 0x0000, new bool[8] { true, true, true, true, true, true, true, true});
            }
        }

        protected void UpdateInputs(bool[] status)
        {
            if (status == null) return;
            Ellipse[] ells = { Ellipse_0, Ellipse_1, Ellipse_2, Ellipse_3, Ellipse_4, Ellipse_5, Ellipse_6, Ellipse_7  };

            int len = Math.Min(ells.Length, status.Length);

            this.Dispatcher.BeginInvoke((Action)delegate ()
                {
                    for (int i = 0; i < len; i++)
                    {
                        ells[i].Fill = status[i] ? Brushes.Red : Brushes.Gray;
                    }
                }
                );
        }

        protected void UpdateOutputs(bool[] status)
        {
            if (status == null) return;
            CheckBox[] boies = { CheckBox_0, CheckBox_1, CheckBox_2, CheckBox_3, CheckBox_4, CheckBox_5, CheckBox_6, CheckBox_7 };
            Rectangle[] rects = { Rectangle_0, Rectangle_1, Rectangle_2, Rectangle_3, Rectangle_4, Rectangle_5, Rectangle_6, Rectangle_7 };

            int len = Math.Min(rects.Length, status.Length);

            this.Dispatcher.BeginInvoke((Action)delegate ()
            {
                for (int i = 0; i < len; i++)
                {
                    boies[i].IsChecked = status[i];
                    rects[i].Fill = status[i] ? Brushes.Red : Brushes.Gray;
                }
            }                           
            );
        }

        private static void ThreadFunction(object obj)
        {
            MainWindow main = (MainWindow)obj;
            while(main.Running)
            {
                if(!main.serialPort.IsOpen)
                {
                    main.Running = false;
                    break;
                }

                bool[] inputs = null, outputs = null;

                try
                {
                    inputs = main.master.ReadInputs(1, 0x0000, 8);
                    outputs = main.master.ReadCoils(1, 0x0000, 8);
                }
                catch(Exception ex)
                {
                    Console.WriteLine(ex.Message);
                    Thread.Sleep(1000);
                }
                
                main.UpdateInputs(inputs);
                main.UpdateOutputs(outputs);

                Thread.Sleep(500);
            }
        }

        private void CheckBox_Click(object sender, RoutedEventArgs e)
        {
            ushort address;
            CheckBox checkBox = (CheckBox)sender;

            if (!ushort.TryParse(checkBox.Name.Replace("CheckBox_", ""), out address))  return;

            try
            {
                master.WriteSingleCoil(1, address, checkBox.IsChecked.Value);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
        }
    }
}
