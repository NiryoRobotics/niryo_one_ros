/*  MIT License

    Copyright (c) 2019 Niryo

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace NiryoOneClient.Tests {
    [TestClass]
    public class HardwareStatusTest {
        [TestMethod]
        public void Parse_Sample_Works () {
            // Arrange
            var s = "[59,2,True,'',0,False,['Stepper Axis 1', 'Stepper Axis 2', 'Stepper Axis 3', 'Servo Axis 4', 'Servo Axis 5', 'Servo Axis 6'],['Niryo Stepper', 'Niryo Stepper', 'Niryo Stepper', 'DXL XL-430', 'DXL XL-430', 'DXL XL-320'],(34, 34, 37, 43, 45, 37),(0.0, 0.0, 0.0, 11.3, 11.2, 7.9),(0, 0, 0, 0, 0, 0)]";
            // Act
            var hs = HardwareStatus.Parse (s);
            // Assert
            Assert.AreEqual (59, hs.RpiTemperature);
            Assert.AreEqual (2, hs.HardwareVersion);
            Assert.AreEqual (true, hs.ConnectionUp);
            Assert.AreEqual ("", hs.ErrorMessage);
            Assert.AreEqual (0, hs.CalibrationNeeded);
            Assert.AreEqual (false, hs.CalibrationInProgress);
            CollectionAssert.AreEqual (new [] { "Stepper Axis 1", "Stepper Axis 2", "Stepper Axis 3", "Servo Axis 4", "Servo Axis 5", "Servo Axis 6" }, hs.MotorNames);
            CollectionAssert.AreEqual (new [] { "Niryo Stepper", "Niryo Stepper", "Niryo Stepper", "DXL XL-430", "DXL XL-430", "DXL XL-320" }, hs.MotorTypes);
            CollectionAssert.AreEqual (new [] { 34, 34, 37, 43, 45, 37 }, hs.Temperatures);
            CollectionAssert.AreEqual (new [] { 0.0m, 0.0m, 0.0m, 11.3m, 11.2m, 7.9m }, hs.Voltages);
            CollectionAssert.AreEqual (new [] { 0, 0, 0, 0, 0, 0 }, hs.HardwareErrors);
        }
    }
}