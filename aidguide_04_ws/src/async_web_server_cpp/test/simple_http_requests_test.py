#!/usr/bin/env python3

import sys
if sys.version_info > (3,):
    import http.client as httplib
else:
    import httplib
import unittest
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
import time

class TestSimpleHttpRequests(unittest.TestCase):
    """Clase TestSimpleHttpRequests.
    
    Implementa funcionalidad para TestSimpleHttpRequests.
    """
    def setUp(self):
        """Función Setup.
        """
        self.conn = httplib.HTTPConnection("localhost:9849")

    def test_ok(self):
        """Función Test ok.
        """
        self.conn.request("GET", "/response/ok")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)

    def test_created(self):
        """Función Test created.
        """
        self.conn.request("GET", "/response/created")
        response = self.conn.getresponse()
        self.assertEqual(201, response.status)

    def test_accepted(self):
        """Función Test accepted.
        """
        self.conn.request("GET", "/response/accepted")
        response = self.conn.getresponse()
        self.assertEqual(202, response.status)

    def test_forbidden(self):
        """Función Test forbidden.
        """
        self.conn.request("GET", "/response/forbidden")
        response = self.conn.getresponse()
        self.assertEqual(403, response.status)

    def test_not_found(self):
        """Función Test not found.
        """
        self.conn.request("GET", "/response/not_found")
        response = self.conn.getresponse()
        self.assertEqual(404, response.status)

    def test_internal_server_error(self):
        """Función Test internal server error.
        """
        self.conn.request("GET", "/response/internal_server_error")
        response = self.conn.getresponse()
        self.assertEqual(500, response.status)

    def test_default_action(self):
        """Función Test default action.
        """
        self.conn.request("GET", "/some_random_url12345")
        response = self.conn.getresponse()
        self.assertEqual(404, response.status)

    def test_default_action2(self):
        """Función Test default action2.
        """
        self.conn.request("GET", "/a_static_response")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual(b"A RESPONSE", response.read())

    def test_http_echo1(self):
        """Función Test http echo1.
        """
        test_content = b"hello HELLO"*1000 # make sure to exceed MTU
        self.conn.request("GET", "/http_body_echo", test_content)
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual(test_content, response.read())

    def test_http_echo2(self):
        """Función Test http echo2.
        """
        test_content = b"THIS is A test"*1000 # make sure to exceed MTU
        self.conn.request("POST", "/http_body_echo", test_content)
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual(test_content, response.read())

    def test_http_path_echo(self):
        """Función Test http path echo.
        """
        self.conn.request("GET", "/http_path_echo/this_is_a_test")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual(b"/http_path_echo/this_is_a_test", response.read())

    def test_http_query_echo(self):
        """Función Test http query echo.
        """
        self.conn.request("GET", "/http_query_echo?hello=1&b=test&c=10")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual(b"b=test\nc=10\nhello=1\n", response.read())

    def test_file(self):
        """Función Test file.
        """
        self.conn.request("GET", "/test_file")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual(b"<html></html>\n", response.read())

    def test_file_from_filesystem(self):
        """Función Test file from filesystem.
        """
        self.conn.request("GET", "/test_files/test_dir/test_file.txt")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual(b"test\n", response.read())

    def test_directory_listing_forbidden_from_filesystem1(self):
        """Función Test directory listing forbidden from filesystem1.
        """
        self.conn.request("GET", "/test_files/test_dir/")
        response = self.conn.getresponse()
        self.assertEqual(403, response.status)

    def test_directory_listing_forbidden_from_filesystem2(self):
        """Función Test directory listing forbidden from filesystem2.
        """
        self.conn.request("GET", "/test_files/test_dir")
        response = self.conn.getresponse()
        self.assertEqual(403, response.status)

    def test_directory_listing_from_filesystem(self):
        """Función Test directory listing from filesystem.
        """
        self.conn.request("GET", "/test_files_with_dir/test_dir/")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)

if __name__ == '__main__':
    time.sleep(1) # ensure server is up

    unittest.main()