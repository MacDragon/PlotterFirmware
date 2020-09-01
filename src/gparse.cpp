//
//  gparse.cpp
//  GCodeParser
//
//  Created by Visa Harvey on 1.9.2020.
//

#include <string>
#include <cstring>
#include <cmath>
#include "gparse.h"

int gettokenlength( const char *input )
{
    while(isspace(input[0])) input++;
    int pos = 0;
    for ( pos=0;(input[pos] != 0) && (input[pos] != ' ' && input[pos] != '\n' && input[pos] != '\r' );pos++);
    return pos;
}


// will only return true if a single positive numeric is found in input.
bool tokenOneNumeric( int32_t * result, const char *input )
{
    // Trim leading space
    while(isspace(input[0])) input++;
    
    int tokenlen = gettokenlength( input );
    
    if ( tokenlen > 0 )
    {
        char token[20] = "";
        
        strncpy(token, input, tokenlen );
        
        int i=0;
        for (;isdigit(token[i]);i++);
        if ( i != tokenlen ) return false; // number of chars in token weere not all digits.
        
        input += tokenlen;
        while(isspace(input[0])) input++;
        *result = atoi(token);
        if ( *result < 0 ) return false; // only allow positive.
        
        if ( strlen( input ) > 0) return false; // only expecting a single numeric result, there's still unparsed data.
    } else
    {
        return false;
    }
    return true;
}

// returns true or false if string tokenised correctly so that input can be reported bad instead.
// expected must be given in order of int32's in tokenise union of command to be generic.
bool tokenScan( int32_t * tokenise, const char *input, const char * expected )
{
    // Trim leading space
    while(isspace(input[0])) input++;

    char found[10] = "";
    size_t foundlen = 0;
    size_t expectedlen= strlen(expected);
    
    // look for wanted tokens.
    while ( input[0] != 0 )
    {
        int32_t * value = nullptr;
        
        const char * expectedpos = strchr(expected, input[0]);
  
        // check if we actually found curret token in expected list, if not, return bad.
        if ( expectedpos == nullptr )
        {
            return false;
        }
        
        // the token has been found in in expected list. Check it's not already been found before.
        
        if ( strchr(found, input[0] ) != nullptr )
        {// we've already seen this token!
            return false;
        }
        
        value = &tokenise[expectedpos-expected];
        
        found[foundlen] = input[0];
        found[foundlen + 1] = 0;
        foundlen++;

        input++;
        
        int tokenlen = gettokenlength(input);
        
        char token[10] = "";
        
        if ( tokenlen > 0 && tokenlen < 10 ) // prevent buffer overflow from overlong token.
        {
            strncpy(token, input, tokenlen ); // take a copt of just current token.
            input += tokenlen;
            
            // looks like x/y are always sent with two digit decimal.
            // lets deal with this by keeping everything as integers and converting value into
            // mm*100 by just removing the period and then converting into int.
            // keeps free of floating point instructions during parse.
            
            long divider = 1;
            
            char * periodpos = strchr(token, '.' ); // search input for a period
            
            if ( periodpos != nullptr )  // if we've found a decimal, lets remove it.
            {
                strncpy(periodpos, periodpos+1, tokenlen-(periodpos-token) );
                tokenlen--;
#ifndef FORCE2DECIMAL
                divider = pow(10, token+tokenlen-(periodpos+2)); // ensure we still keep minimum of 2 decimals in answer precision
#else
                if ( token+tokenlen-periodpos != 2 ) // if we have a period we must have two numerics after it.
                    return false; // check there's exactly two digits after period.
#endif
            }
                        
            int i = 0;
            
            if ( token[0] == '-' ) i = 1; // accomodate for negative value jumping sign
            
            for (;isdigit(token[i]);i++); // chest remaining charecters of token to ensure fully numeric.
            
            if ( i != tokenlen ) return false; // number of chars in token were not all digits.
            
            // workaround to ensure that the two variables that should have fractional component will
            if ( ( expectedpos[0] == 'X' || expectedpos[0] == 'Y' )
                && divider == 1 && periodpos == nullptr )
                *value = atoi(token) * 100;
            else if ( divider < 1)
                *value = atoi(token) * 10;
            else
                *value = atoi(token) / divider;
        } else
        {
            return false;
        }
        
        while(isspace(input[0])) input++;
    }
    
    if ( expectedlen != foundlen) return false; // not found enough tokens to satisfy
    // no other tokens expected.
    return true;
}


// make a check that there is no more information left in input.
bool checkNoTokens( const char *input)
{
    while(isspace(input[0])) input++;
    
    if ( strlen(input) ==0 ) return true;
    else return false;
}

command GCodeParser( const char *input )
{
    command res;
    res.cmd = bad; // set default return to bad, so we assume incorrect data till proved otherwise.
    
 //   char * instrlocal = instr; // cpy[81] = ""; // max line length,
    
//    char * input = instrcpy;
    
 //   strncpy(input, instr, 80); // create a local copy of the string so we can do whatever we want with it.
    
    char token[5] = ""; // setup token input
    int tokenint = 0;
 //   int inputstart = 0;
    
    while(isspace(input[0])) input++; // Trim leading space by moving pointer forward till no whitespace left or end of string.

    if(input[0] == 0)  // if we are at end of string return as empty string, could also return as bad, but empty string is different to bad input.
    {
        res.cmd=none;
        return res;
    }

    // at this point we have a sanitised string with white space removed.
    
    bool mcommand = false; // determine if we have an M or G command from first letter.
        
    switch ( input[0] ){
        case 'M' :
            input++;
            mcommand = true;
            break;  // init functions
        case 'G' :
            input++;
            break;
        default : // unecpected command.
            res.cmd = bad;
            return res;
    }
    
    int tokenlen = gettokenlength(input); // check length of rest of command token after letter.
    
    if ( tokenlen > 0 && tokenlen < 5 ) // assuming we do have something after letter, check it
    {
        strncpy(token, input, tokenlen );
        input += tokenlen;
        
        tokenint = atoi(token);
        
        if ( tokenint > 0 ) // we got a conversion, 0 is not in expected command sets so a failed atoi with non numerics will fail out.
        {
           if ( mcommand ) // process rest of input line with relevant parser.
           { // M Commands
               switch ( tokenint ) {
                   case 10 : // info dump // these commands should have no additional input.
                       if ( checkNoTokens(input) )
                           res.cmd = init;
                       break;
                   case 11 : // limit switches
                       if ( checkNoTokens(input) )
                           res.cmd = limit;
                       break;
                   case 2 : // store pen positions // expects two lettered tokens
                       if ( tokenScan( (int32_t *) &res.penstore, input, "UD" ) )
                           res.cmd = savepen;
                       break;
                   case 1 : // set pen position. // expects a single number without letter code.
                       if ( tokenOneNumeric( (int32_t *) &res.pen.pos, input ) )
                           res.cmd = setpen;
                       break;
                   case 5 : // save stepper direction, area, speed. // expects 5 lettered tokens.
                       if ( tokenScan( (int32_t *) &res.penstore, input, "ABHWS" ) )
                           res.cmd = savestepper;
                       break;
                   case 4 : // laser power.
                       if ( tokenOneNumeric( (int32_t *) &res.laser.power, input ) )
                           res.cmd = setlaser;
                       break;
                   default : // unknown command code
                       res.cmd = bad;
               }
           } else
           { // G(o) Commands
               switch ( tokenint ) {
                   case 1 : // go xy
                       if ( tokenScan( (int32_t *) &res.penstore, input, "XYA" ) ) res.cmd = goxy;
                       break;
                   case 28 : //  go origin, no data. -- this could also probably be stored as go x0, y0, abs 1
                       if ( checkNoTokens(input) )
                           res.cmd = origin;
                       break;
                   default : // unknown command code
                       res.cmd = bad;
               }
            }
            return res;
        }
    }
    
  /*  if ( strcmp(input, "Init") == 0)
        res.cmd = init;
    else res.cmd = bad;
*/
    return res;
}
