
const char* edit_form = "<html>\
<title>\
Parakeet-8266 Settings\
</title>\
<body> \
<form action='save' method='POST'>\
<table  border='0' cellpadding='2' cellspacing='2' style='border-collapse: collapse' width='100%'>\
<tr>\
  <td align=center>\
    Parakeet-8266 Settings\
  </td>\
</tr>\
<tr>\
  <td width=30%>\
    Transmitter ID:\
  </td>\
  <td align=left>\
   <input type='text' name='DexcomID' maxlength='5' size=5 value=\"%s\">\
  </td>\
</tr>\
<tr>\
  <td width=30%>\
    Password Code:\
  </td>\
  <td>\
    <input type='text' name='PasswordCode' maxlength='5' size='5'  value=\"%s\">\
  </td>\
</tr>\
<tr>\
  <td width=30%>\
    Webservice URL:\
  </td>\
  <td>\
    <input type='text' name='WebService' maxlength='55' size='55' value=\"%s\">\
  </td>\
</tr>\
<tr>\
  <td align=center>\
    <input type='submit' value='Save' />\
  </td>\
</tr>\
</table>\
</form>\
</body>\
</html>";
