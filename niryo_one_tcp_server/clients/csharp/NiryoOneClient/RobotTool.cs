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

namespace NiryoOneClient
{
    /// <summary>
    /// An enumeration of the known types of tools that can be connected to the Niryo One robotic arm
    /// </summary>
    public enum RobotTool
    {
        /// <summary>The first type of gripper</summary>
        GRIPPER_1,
        /// <summary>The second type of gripper</summary>
        GRIPPER_2,
        /// <summary>The third type of gripper</summary>
        GRIPPER_3,
        /// <summary>A vacuum pump</summary>
        VACUUM_PUMP_1,
        /// <summary>An electromagnet</summary>
        ELECTROMAGNET_1
    }
}