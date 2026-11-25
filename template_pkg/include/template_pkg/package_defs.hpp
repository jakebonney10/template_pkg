// Copyright 2024 Seaward Science
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#ifndef TEMPLATE_PKG__PACKAGE_DEFS_HPP_
#define TEMPLATE_PKG__PACKAGE_DEFS_HPP_

///
///  Namespace stuff
///
/// These macros define the namespace of the package in one place.  Each
/// definition must stay on a single line inside the INDENT-OFF block so the
/// ROS2 formatter (uncrustify) can parse this file.
///

/* *INDENT-OFF* */
#define NS_HEAD namespace template_pkg {
#define NS_FOOT }
/* *INDENT-ON* */

#endif  // TEMPLATE_PKG__PACKAGE_DEFS_HPP_
