function sendEmail(configFile, title, msg, attach)
%SENDEMAIL Sends an email to a specified email address. Can use this to send status udpates
%or send files.
    
    % Read the data from the config file
    fid = fopen(configFile, 'r');
    email = fgetl(fid);
    password = fgetl(fid);
    fclose(fid);
    
    setpref('Internet','SMTP_Server','smtp.gmail.com');
    setpref('Internet','E_mail', email);
    setpref('Internet','SMTP_Username', email);
    setpref('Internet','SMTP_Password', password);

    props = java.lang.System.getProperties;
    props.setProperty('mail.smtp.auth', 'true');
    props.setProperty('mail.smtp.socketFactory.class', 'javax.net.ssl.SSLSocketFactory');
    props.setProperty('mail.smtp.socketFactory.port', '465');
    
    % Variation of calling function based on number of arguments
    if nargin == 3
        sendmail(email, title);
    elseif nargin  == 4
        sendmail(email, title, msg);
    elseif nargin == 5
        sendmail(email, title, msg, {attach});
    end
    setpref('Internet','SMTP_Server','');
    setpref('Internet','E_mail','');
    setpref('Internet','SMTP_Username','');
    setpref('Internet','SMTP_Password','');
end