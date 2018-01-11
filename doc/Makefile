FTEX=documentation

all	:
	pdflatex ${FTEX}.tex

clean: 
	rm -f ${FTEX}.bbl ${FTEX}.blg  ${FTEX}.aux ${FTEX}.dvi ${FTEX}.log ${FTEX}.out *~ ${FTEX}.pdf


view: ${FTEX}.pdf
	evince ${FTEX}.pdf
