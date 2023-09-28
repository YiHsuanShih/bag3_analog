module high_pass_mdl(
        //input
	    input real in,
        input real bias,
        //output
        output real out
);

parameter real e = 2.7182;
parameter Tau = {{ Tau | default(100, true) }};
parameter Ts = {{ Ts | default(1, true) }};
//parameter real e_wT = e**SAMPLE_TIME/Tau;
   
real        in_1, bias_1, out_1;

   
real  in_prev, bias_prev, out_prev, t_prev;
real  din, bias_term, prev_term, exp_settle, step_settle;   


initial begin
   out = bias;
   in_prev = in;
   bias_prev = bias;
   din = 0;
end

always @(in or bias) begin
   in_prev <= in;  
   bias_prev <= bias;   
   out <= out*e**(-($realtime-t_prev)/Tau) + in-in_prev + 
	  bias_prev*(1-e**(-($realtime-t_prev)/Tau));
   t_prev <= $realtime;
end
endmodule

{{ _header }}
high_pass_mdl  HPFp(
		   .in(inp),
		   .bias(biasp),
		   .out(outp)
		   );

high_pass_mdl HPFn(
		   .in(inn),
		   .bias(biasn),
		   .out(outn)
		   );
endmodule